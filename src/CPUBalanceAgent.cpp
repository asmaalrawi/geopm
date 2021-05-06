/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"

#include <cmath>
#include <cassert>
#include <algorithm>

#include "PluginFactory.hpp"
#include "PlatformIO.hpp"
#include "PlatformTopo.hpp"
#include "Helper.hpp"
#include "Agg.hpp"
#include "CircularBuffer.hpp"
#include "geopm_debug.hpp"

#include <string> 

#include "CPUBalanceAgent.hpp"

namespace geopm
{

    CPUBalanceAgent::CPUBalanceAgent()
        : CPUBalanceAgent(platform_io(), platform_topo())
    {

    }

    CPUBalanceAgent::CPUBalanceAgent(PlatformIO &plat_io, const PlatformTopo &topo)
        : m_platform_io(plat_io)
          , m_platform_topo(topo)
          , m_level(-1)
          , m_num_children(0)
          , M_FREQ_STEP(1e8) 
          , M_WAIT_SEC(0.01) 
          , m_buffer_size(5)
          , m_num_cpu(m_platform_topo.num_domain(GEOPM_DOMAIN_CPU))
          , m_num_core(m_platform_topo.num_domain(GEOPM_DOMAIN_CORE))
          , m_num_pkg(m_platform_topo.num_domain(GEOPM_DOMAIN_PACKAGE))
          , m_num_iter(0)
          , m_initialized_freq(false)
          , m_signal_idx(m_num_core, std::vector<int>(M_NUM_SIGNAL))  
          , m_control_idx(m_num_core, std::vector<int>(M_NUM_CONTROL))  
          , m_sample_new(m_num_core, std::vector<double>(M_NUM_SIGNAL))
          , m_sample_hist(m_num_core, std::vector<double>(M_NUM_SIGNAL))
          , m_freq(m_num_core, 0)
          , m_clos(m_num_core, 0)
          , m_last_wait{{0,0}}
          , m_last_balance{{0,0}}
          , m_do_send_policy(true)
          , m_do_write_batch(false)
          , m_freq_prog(m_num_core)
          , m_scale_factor(1E9)
          , m_core_freq_min(NAN)
          , m_core_freq_max(NAN)
        {
            geopm_time(&m_last_wait);
            geopm_time(&m_last_balance);
        }

    std::string CPUBalanceAgent::plugin_name(void)
    {
        return "cpu_balance";
    }

    std::unique_ptr<Agent> CPUBalanceAgent::make_plugin(void)
    {
        return geopm::make_unique<CPUBalanceAgent>();
    }

    void CPUBalanceAgent::init(int level, const std::vector<int> &fan_in, bool is_root)
    {
        m_level = level;
        if(level == 0){
            m_num_children = 0;
            init_platform_io();
        }
        else{
            m_num_children = fan_in[level - 1];
        }
    }
    void CPUBalanceAgent::validate_policy(std::vector<double> &in_policy) const
    {
        GEOPM_DEBUG_ASSERT(in_policy.size() == M_NUM_POLICY,
                "CPUBalanceAgent::" + std::string(__func__) +
                "(): policy vector not correctly sized.");

        if (is_all_nan(in_policy)) {
            // All-NAN policy may be received before the first policy
            /// @todo: in the future, this should not be accepted by this agent.
            return;
        }

        //Required policy settings
        if(std::isnan(in_policy[M_POLICY_REBALANCE_ON])){
            throw Exception("CPUBalanceAgent::" + std::string(__func__) +
                    "(): policy parameter M_POLICY_REBALANCE_ON must be specified.",
                    GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        //Default values
        if(std::isnan(in_policy[M_POLICY_FREQUENCY_MAX])){
            in_policy[M_POLICY_FREQUENCY_MAX] = m_core_freq_max;
        }
        if(std::isnan(in_policy[M_POLICY_FREQUENCY_MIN])){
            in_policy[M_POLICY_FREQUENCY_MIN] = m_core_freq_min;
        }

        //TODO: Only set these if rebalance is on
        if(std::isnan(in_policy[M_POLICY_HISTORY_WEIGHT])){
            in_policy[M_POLICY_HISTORY_WEIGHT] = 0.95;
        }
        if(std::isnan(in_policy[M_POLICY_CTRL_DELAY_ITER])){
            in_policy[M_POLICY_CTRL_DELAY_ITER] = 5.0;
        }
        if(std::isnan(in_policy[M_POLICY_CTRL_RATE_SEC])){
            in_policy[M_POLICY_CTRL_RATE_SEC] = M_WAIT_SEC;
        }

        //Range clipping values
        if (in_policy[M_POLICY_FREQUENCY_MIN] < m_core_freq_min){
            throw Exception("CPUBalanceAgent::" + std::string(__func__) +
                    "(): policy max frequency out of range: " +
                    std::to_string(in_policy[M_POLICY_FREQUENCY_MIN]) + ".",
                    GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if(in_policy[M_POLICY_HISTORY_WEIGHT] > 1.0 || 
                in_policy[M_POLICY_HISTORY_WEIGHT] < 0.0){
            throw Exception("CPUBalanceAgent::" + std::string(__func__) +
                    "(): policy history weight out of range [0-1]: " +
                    std::to_string(in_policy[M_POLICY_HISTORY_WEIGHT]) + ".",
                    GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if(in_policy[M_POLICY_CTRL_RATE_SEC] < M_WAIT_SEC){
            in_policy[M_POLICY_CTRL_RATE_SEC] = M_WAIT_SEC;
        }

    }
    bool CPUBalanceAgent::is_all_nan(const std::vector<double> &vec)
    {
        return std::all_of(vec.begin(), vec.end(),
                [](double x) -> bool { return std::isnan(x); });
    }


    void CPUBalanceAgent::split_policy(const std::vector<double>& in_policy,
            std::vector<std::vector<double> >& out_policy)
    {
        throw Exception("CPUBalanceAgent::" + std::string(__func__) + 
                "() Should not be running on multiple nodes", 
                GEOPM_ERROR_NOT_IMPLEMENTED,
                __FILE__, __LINE__);
        //TODO: Look at power governor agent and copy logic/tests from there
        GEOPM_DEBUG_ASSERT(in_policy.size() == M_NUM_POLICY,
                "CPUBalanceAgent::" + std::string(__func__) +
                "(): policy vector not correctly sized.");
        std::fill(out_policy.begin(), out_policy.end(), in_policy);
        //for (auto &child_pol : out_policy) {
        //    child_pol = in_policy;
        //}
    }
    bool CPUBalanceAgent::do_send_policy(void) const
    {
        //TODO: Send policy only once in beginning
        return m_do_send_policy;
    }
    void CPUBalanceAgent::aggregate_sample(const std::vector<std::vector<double> > &in_sample,
            std::vector<double>& out_sample)
    {
        //TODO: Decide [if desired] how to extend to multiple nodes

    }
    bool CPUBalanceAgent::do_send_sample(void) const
    {
        return false;
    }
    void CPUBalanceAgent::adjust_platform(const std::vector<double> &in_policy)
    {
        m_do_write_batch=false;
        std::fill(m_freq.begin(), m_freq.end(), in_policy[M_POLICY_FREQUENCY_MAX]);

        //Do not rebalance unless in a region where thread progress is reported
        if(! in_region()){
            return;
        }
        //Do not rebalance unless enough time has passed (based on policy input)
        if(geopm_time_since(&m_last_balance) < in_policy[M_POLICY_CTRL_RATE_SEC]){
            return;
        }

        //Do not adjust unless rebalance is on and until enough samples have occured
        if(in_policy[M_POLICY_REBALANCE_ON] == 1.0 && m_num_iter >= (int)(in_policy[M_POLICY_CTRL_DELAY_ITER])) {
            //Use CLOS to set priority
            //Rebalance without historical data
            if(in_policy[M_POLICY_HISTORY_WEIGHT] == 0.0){
                if(in_policy[M_POLICY_USE_SST] == 1.0){
                    distribute_clos();
                }
                distribute_freq_linearly_ttc(in_policy[M_POLICY_FREQUENCY_MIN], 
                        in_policy[M_POLICY_FREQUENCY_MAX]);
            }
            //Rebalance while gathering fit history
            else{
                update_fit(in_policy[M_POLICY_HISTORY_WEIGHT]);
                //TODO: Add error filtering?
                //Try to aim slightly better than worst case?
                distribute_freq_from_regression(in_policy[M_POLICY_FREQUENCY_MIN], 
                        in_policy[M_POLICY_FREQUENCY_MAX], in_policy[M_POLICY_USE_SST]);
            }
        }

        geopm_time(&m_last_balance);
        set_all_cpu_settings(in_policy[M_POLICY_FREQUENCY_MAX], in_policy[M_POLICY_USE_SST]);
        m_num_iter++;
        return;
    }

    bool CPUBalanceAgent::do_write_batch(void) const
    {
        return m_do_write_batch;
    }
    bool CPUBalanceAgent::in_region()
    {

        //If any cores are showing progress, it is in region
        for(int core_idx=0; core_idx < m_num_core; core_idx++)
            if( ! std::isnan(m_sample_new[core_idx][M_SIGNAL_PROGRESS]))
                return true;
        return false;
    }
    bool CPUBalanceAgent::skip_adjustment(int core_idx)
    {
        if(std::isnan(m_sample_new[core_idx][M_SIGNAL_PROGRESS])) 
            return true;
        //TODO: If delta progress is 0, extrapolate what it would have been
        //      Remove from skip_adjustment
        if(m_sample_new[core_idx][M_SIGNAL_PROGRESS] <= m_sample_hist[core_idx][M_SIGNAL_PROGRESS]) 
            return true;
        //Occurs if work is redistributed to the thread
        //if(m_sample_new[core_idx][M_SIGNAL_PROGRESS] >= 1.0)
        //    return true;
        return false;
    } 
    //Simple linear distribution based on progress
    void CPUBalanceAgent::distribute_freq_linearly_ttc(double freq_min, double freq_max)
    {
        double longest_time_to_complete = 0.0;
        std::vector<double> est_ttc(m_num_core, 0);
        std::vector<double> d_prog(m_num_core, 0);

        for(int core_idx=0; core_idx < m_num_core; core_idx++){
            if(skip_adjustment(core_idx)) continue;
            d_prog[core_idx] = m_sample_new[core_idx][M_SIGNAL_PROGRESS] - m_sample_hist[core_idx][M_SIGNAL_PROGRESS];
            if(d_prog[core_idx] == 0.0) continue;

            //    timesteps to completion at max frequency

            est_ttc[core_idx] = (m_sample_new[core_idx][M_SIGNAL_FREQ]/freq_max)
                * (1.0-m_sample_new[core_idx][M_SIGNAL_PROGRESS])
                /d_prog[core_idx];
            if(longest_time_to_complete < est_ttc[core_idx])
                longest_time_to_complete = est_ttc[core_idx];
        }

        for(int core_idx=0; core_idx < m_num_core; core_idx++){
            if(skip_adjustment(core_idx) || d_prog[core_idx] == 0.0) {
                m_freq[core_idx] = freq_max; 
            }
            else{
                m_freq[core_idx] = est_ttc[core_idx] / longest_time_to_complete * freq_max;
            }
        }

        return;
    }
    void CPUBalanceAgent::distribute_freq_linearly_prog(double freq_min, double freq_max)
    {
        double worst_progress = 1.0;
        std::vector<double> est_progress(m_num_core, 0);
        std::vector<double> d_prog(m_num_core, 0);

        for(int core_idx=0; core_idx < m_num_core; core_idx++){
            if(skip_adjustment(core_idx)) continue;
            d_prog[core_idx] = m_sample_new[core_idx][M_SIGNAL_PROGRESS] - m_sample_hist[core_idx][M_SIGNAL_PROGRESS];

            est_progress[core_idx] = m_sample_new[core_idx][M_SIGNAL_PROGRESS]
                + d_prog[core_idx] * freq_max / m_sample_new[core_idx][M_SIGNAL_FREQ];
            if(worst_progress < est_progress[core_idx])
                worst_progress = est_progress[core_idx];
        }

        for(int core_idx=0; core_idx < m_num_core; core_idx++){
            if(skip_adjustment(core_idx) || d_prog[core_idx] == 0) {
                m_freq[core_idx] = freq_max; 
            }
            else{
                m_freq[core_idx] = (worst_progress - m_sample_new[core_idx][M_SIGNAL_PROGRESS])
                    * freq_max / d_prog[core_idx];
            }
        }

        return;
    }

    void CPUBalanceAgent::distribute_clos()
    {
        std::vector<double> core_progress;
        core_progress.reserve(m_num_core);

        for(int core_idx=0; core_idx<m_num_core; core_idx++){
            if(std::isnan(m_sample_new[core_idx][M_SIGNAL_PROGRESS])){
                core_progress.push_back(0);
            }
            else{
                core_progress.push_back(m_sample_new[core_idx][M_SIGNAL_PROGRESS]);
            }
        }
        std::sort(core_progress.begin(), core_progress.end());

        double median_progress = core_progress[16];
        double first_q_prog = core_progress[8];
        double third_q_prog = core_progress[3*m_num_core/4];


        for(int core_idx=0; core_idx<m_num_core; core_idx++){
            if(skip_adjustment(core_idx)) continue;
            double core_progress = m_sample_new[core_idx][M_SIGNAL_PROGRESS];

            m_clos[core_idx] = 3;
            if(core_progress < first_q_prog) m_clos[core_idx] = 0;
            else if(core_progress < median_progress) m_clos[core_idx] = 1;
            else if(core_progress < third_q_prog) m_clos[core_idx] = 2;

        }
        return;
    }
    void CPUBalanceAgent::distribute_freq_simply(double freq_min, double freq_max)
    {
        double median_progress = 0.0, min_progress = 0.0, max_progress = 1.0;

        double median_freq = 0.5*(freq_max + freq_min);
        double first_q_freq = 0.5* (freq_min + median_freq);
        double third_q_freq = 0.5* (freq_max + median_freq);

        for(int core_idx=0; core_idx<m_num_core; core_idx++){
            if(skip_adjustment(core_idx)) continue;
            double core_progress = m_sample_new[core_idx][M_SIGNAL_PROGRESS];
            median_progress += core_progress;
            if(min_progress > core_progress)
                min_progress = core_progress;
            if(max_progress < core_progress)
                max_progress = core_progress;
        }

        median_progress = median_progress / m_num_core;
        double first_q_prog = 0.5*(min_progress + median_progress);
        double third_q_prog = 0.5*(median_progress + max_progress);


        for(int core_idx=0; core_idx<m_num_core; core_idx++){
            if(skip_adjustment(core_idx)) continue;
            double core_progress = m_sample_new[core_idx][M_SIGNAL_PROGRESS];

            if(core_progress < first_q_prog) m_freq[core_idx] = freq_max;
            else if(core_progress < median_progress) m_freq[core_idx] = third_q_freq;
            else if(core_progress < third_q_prog) m_freq[core_idx] = first_q_freq;
            else m_freq[core_idx] = freq_min;

        }
        return;

    }
    //Determine frequency based on online least-squares regression
    void CPUBalanceAgent::distribute_freq_from_regression(double freq_min, double freq_max, double use_sst)
    {

        double worst_max_prog = -5;


        //Find target progress
        for(int core_idx=0; core_idx<m_num_core; core_idx++){
            if(skip_adjustment(core_idx)) continue;

            //Best progress each core could achieve in next step (max freq)
            double best_prog_estimate = m_sample_new[core_idx][M_SIGNAL_PROGRESS] 
                + m_freq_prog[core_idx].estimate_measure(freq_max/m_scale_factor);
            if(worst_max_prog < 0) 
                worst_max_prog = best_prog_estimate;

            worst_max_prog = worst_max_prog > best_prog_estimate ? 
                best_prog_estimate : worst_max_prog;
        }

        for(int core_idx=0; core_idx<m_num_core; core_idx++){
            if(skip_adjustment(core_idx)) continue;

            double dprog_desired = worst_max_prog - m_sample_new[core_idx][M_SIGNAL_PROGRESS];
            m_freq[core_idx] = m_scale_factor * m_freq_prog[core_idx].estimate_ctrl(dprog_desired);
            m_freq[core_idx] = m_freq[core_idx] > freq_max ? freq_max : m_freq[core_idx];
            m_freq[core_idx] = m_freq[core_idx] < freq_min ? freq_min : m_freq[core_idx];

        }
        //TODO: Fix this, it's disgusting. Math is bad. Hardcoded buckets. Ew.
        if(use_sst == 1.0){
            std::vector<double> core_freq;
            core_freq.reserve(m_num_core);
            for(int core_idx=0; core_idx<m_num_core; core_idx++){
                if(skip_adjustment(core_idx)){
                    core_freq.push_back(0);
                }
                else{
                    core_freq.push_back(m_freq[core_idx]);
                }
            }
            std::sort(core_freq.begin(), core_freq.end());
            double median_freq = core_freq[16];
            double first_q_prog = core_freq[8];
            double third_q_prog = core_freq[3*m_num_core/4];


            for(int core_idx=0; core_idx<m_num_core; core_idx++){

                m_clos[core_idx] = 3;
                if(m_freq[core_idx] < first_q_prog) m_clos[core_idx] = 0;
                else if(m_freq[core_idx] < median_freq) m_clos[core_idx] = 1;
                else if(m_freq[core_idx] < third_q_prog) m_clos[core_idx] = 2;
            }

        }

        return;
    }
    //Read in platform signals
    void CPUBalanceAgent::sample_platform(std::vector<double> &out_sample)
    {
#ifdef GEOPM_DEBUG
        if(out_sample.size() != M_NUM_SIGNAL){
            throw Exception("CPUBalanceAgent::" + std::string(__func__) 
                    + "(): out_sample vector not correctly sized.",
                    GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
#endif
        for(size_t sample_idx = 0; sample_idx < M_NUM_SIGNAL; ++sample_idx){
            out_sample[sample_idx] = 0;
            for(int core_idx=0; core_idx<m_num_core; core_idx++){
                m_sample_hist[core_idx][sample_idx] = m_sample_new[core_idx][sample_idx];
                m_sample_new[core_idx][sample_idx] = m_platform_io.sample(m_signal_idx[core_idx][sample_idx]);
                out_sample[sample_idx]+= m_sample_new[core_idx][sample_idx];
                //printf("DEBUG: Sample idx %d core_idxore %d  %f -> %f\n", sample_idx, core_idx, m_sample_hist[core_idx][sample_idx], m_sample_new[core_idx][sample_idx]);
            }
            out_sample[sample_idx]=out_sample[sample_idx]/m_num_core;
        }


        //TODO: Aggregate (min) per-thread progress to per-core

        return;
    }
    void CPUBalanceAgent::wait(void)
    {
        while(geopm_time_since(&m_last_wait) < M_WAIT_SEC){
        }

        geopm_time(&m_last_wait);
    }
    std::vector<std::string> CPUBalanceAgent::policy_names(void)
    {
        return {"REBALANCE_ON", "USE_SST", "FREQUENCY_MAX", "FREQUENCY_MIN",
            "HISTORY_WEIGHT", "CTRL_DELAY_ITER", "CTRL_RATE_SEC"};
    }

    std::vector<std::string> CPUBalanceAgent::sample_names(void)
    {
        return {"APERF", "MPERF", "FREQ_MEAN", "PROGRESS_MEAN"};
    }
    // Adds the wait time to the top of the report
    //Add other system-wide agent-related info here
    std::vector<std::pair<std::string, std::string> > CPUBalanceAgent::report_header(void) const
    {
        return {}; 
    }
    std::vector<std::pair<std::string, std::string> > CPUBalanceAgent::report_host(void) const
    {
        return {};
    }
    std::map<uint64_t, std::vector<std::pair<std::string, std::string> > > 
        CPUBalanceAgent::report_region(void) const
        {
            return {};
        }
    std::vector<std::string> CPUBalanceAgent::trace_names(void) const
    {
        std::vector<std::string> names; 
        std::string tmp_s, scal_s;

        //TODO: Determine which of these are required in final agent
        //      Remember also to change trace_values
        for(int core_idx=0; core_idx<m_num_core; core_idx++){
            tmp_s = "frequency_c" + std::to_string(core_idx);
            names.push_back(tmp_s);
        }
        // for(int core_idx=0; core_idx<m_num_core; core_idx++){
        //     tmp_s = "freq_request_c" + std::to_string(core_idx);
        //     names.push_back(tmp_s);
        // }
        for(int core_idx=0; core_idx<m_num_core; core_idx++){
            tmp_s = "progress_c" + std::to_string(core_idx);
            names.push_back(tmp_s);
        }
        // for(int core_idx=0; core_idx<m_num_core; core_idx++){
        //     tmp_s = "perf_c" + std::to_string(core_idx);
        //     names.push_back(tmp_s);
        // }
        // for(int core_idx=0; core_idx<m_num_core; core_idx++){
        //     tmp_s = "slope_c" + std::to_string(core_idx);
        //     names.push_back(tmp_s);
        // }
        // for(int core_idx=0; core_idx<m_num_core; core_idx++){
        //     tmp_s = "intercept_c" + std::to_string(core_idx);
        //     names.push_back(tmp_s);
        // }
        //for(int core_idx=0; core_idx<m_num_pkg; core_idx++){
        //    tmp_s = "power_p" + std::to_string(core_idx);
        //    names.push_back(tmp_s);
        //}

        return names;
    }
    std::vector<std::function<std::string(double)> >CPUBalanceAgent::trace_formats(void) const
    {
        return{};
    }
    void CPUBalanceAgent::trace_values(std::vector<double> &values)
    {
        //TODO: Add pkg power to trace values

        for(int core_idx=0; core_idx<m_num_core; core_idx++){
            //int pkg_id = m_platform_topo.domain_idx(GEOPM_DOMAIN_PACKAGE, core_idx);

            values[core_idx] = m_sample_new[core_idx][M_SIGNAL_FREQ];
            //values[m_num_core+core_idx] = m_freq[core_idx];
            //values[2*m_num_core+core_idx] = m_sample_new[core_idx][M_SIGNAL_PROGRESS];
            values[m_num_core+core_idx] = m_sample_new[core_idx][M_SIGNAL_PROGRESS];
            //TODO: Don't hardcode sticker, you can read it from GEOPM somewhere
            // values[3*m_num_core+core_idx] = 
            //     2.1E9*(m_sample_new[core_idx][M_SIGNAL_APERF]-m_sample_hist[core_idx][M_SIGNAL_APERF])
            //     /(m_sample_new[core_idx][M_SIGNAL_MPERF]-m_sample_hist[core_idx][M_SIGNAL_MPERF]);

            // if(m_num_iter > 1){
            //         std::vector<double> state_vector = m_freq_prog[core_idx].multiply_vector_by_inverse_matrix();
            //         values[4*m_num_core+core_idx] = state_vector[0];
            //         values[5*m_num_core+core_idx] = state_vector[1];
            // }
            // else{
            //         values[4*m_num_core+core_idx] = 0;
            //         values[5*m_num_core+core_idx] = 0;
            // }
            //values[2*m_num_core+pkg_id] = m_sample_new[core_idx][M_SIGNAL_PKG_POWER];
        }
    }
    void CPUBalanceAgent::enforce_policy(const std::vector<double> &policy) const
    {
        if(policy.size() != M_NUM_POLICY){
            throw Exception("CPUAgent::enforce_policy(): policy vector incorrectly sized.",
                    GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
    }
    void CPUBalanceAgent::update_fit(double hist_weight){

        for(int core_idx=0; core_idx<m_num_core; core_idx++){
            if(skip_adjustment(core_idx)) continue;

            double dprog = m_sample_new[core_idx][M_SIGNAL_PROGRESS]
                - m_sample_hist[core_idx][M_SIGNAL_PROGRESS];

            m_freq_prog[core_idx].update_vector(m_sample_new[core_idx][M_SIGNAL_FREQ]/m_scale_factor
                    , dprog, hist_weight);
            m_freq_prog[core_idx].update_fit(m_sample_new[core_idx][M_SIGNAL_FREQ]/m_scale_factor
                    , dprog, hist_weight);
        }
        return;
    }

    void CPUBalanceAgent::set_all_cpu_settings(double freq_max, double use_sst)
    {

        if(use_sst == 1.0){
            for(int core_idx = 0; core_idx < m_num_core; core_idx++){
                m_platform_io.adjust(m_control_idx[core_idx][M_CONTROL_SETTING],
                        m_clos[core_idx]);
                //m_platform_io.write_control("SST::COREPRIORITY:ASSOCIATION", 
                //								GEOPM_DOMAIN_CORE, core_idx, m_clos[core_idx]);
            }
        }
        else{
            for(int core_idx = 0; core_idx < m_num_core; core_idx++){
                if(std::isnan(m_freq[core_idx])){
                    m_freq[core_idx] = freq_max;
                }
                //Only adjust if the frequency has changed. 
                //      Note: Currently commented out to debug strange frequency behavior
                // TODO: Debug frequency behavior and replace this line
                //else if(m_freq[core_idx] != m_sample_new[core_idx][M_SIGNAL_FREQ]){
                m_platform_io.adjust(m_control_idx[core_idx][M_CONTROL_SETTING], m_freq[core_idx]);
                // }
            }
        }
        m_do_write_batch = true;
        return;
    }

    void CPUBalanceAgent::init_platform_io(void)
    {

        //TODO: Future--determine max uncore freq
        m_platform_io.write_control("MSR::UNCORE_RATIO_LIMIT:MAX_RATIO", GEOPM_DOMAIN_BOARD, 
                0, 16e9);
        m_platform_io.write_control("MSR::UNCORE_RATIO_LIMIT:MIN_RATIO", GEOPM_DOMAIN_BOARD, 
                0, 16e9);

        //TODO: Do not hardcode all this stuff
        m_platform_io.write_control("SST::COREPRIORITY:0:WEIGHT", GEOPM_DOMAIN_BOARD, 0, 0);
        m_platform_io.write_control("SST::COREPRIORITY:1:WEIGHT", GEOPM_DOMAIN_BOARD, 0, 5);
        m_platform_io.write_control("SST::COREPRIORITY:2:WEIGHT", GEOPM_DOMAIN_BOARD, 0, 10);
        m_platform_io.write_control("SST::COREPRIORITY:3:WEIGHT", GEOPM_DOMAIN_BOARD, 0, 15);

        m_platform_io.write_control("SST::COREPRIORITY:0:FREQUENCY_MIN", GEOPM_DOMAIN_BOARD, 0, 3.0e9);
        m_platform_io.write_control("SST::COREPRIORITY:1:FREQUENCY_MIN", GEOPM_DOMAIN_BOARD, 0, 2.4e9);
        m_platform_io.write_control("SST::COREPRIORITY:2:FREQUENCY_MIN", GEOPM_DOMAIN_BOARD, 0, 2.0e9);
        m_platform_io.write_control("SST::COREPRIORITY:3:FREQUENCY_MIN", GEOPM_DOMAIN_BOARD, 0, 1.0e9);

        m_platform_io.write_control("SST::COREPRIORITY:0:FREQUENCY_MAX", GEOPM_DOMAIN_BOARD, 0, 4.0e9);
        m_platform_io.write_control("SST::COREPRIORITY:1:FREQUENCY_MAX", GEOPM_DOMAIN_BOARD, 0, 4.0e9);
        m_platform_io.write_control("SST::COREPRIORITY:2:FREQUENCY_MAX", GEOPM_DOMAIN_BOARD, 0, 4.0e9);
        m_platform_io.write_control("SST::COREPRIORITY:3:FREQUENCY_MAX", GEOPM_DOMAIN_BOARD, 0, 4.0e9);

        m_core_freq_min = m_platform_io.read_signal("FREQUENCY_MIN", GEOPM_DOMAIN_BOARD, 0);
        m_core_freq_max = m_platform_io.read_signal("FREQUENCY_MAX", GEOPM_DOMAIN_BOARD, 0);
        //TODO: Size vectors appropriately, reverse [signal][domain]
        for(int core_idx=0; core_idx<m_num_core; core_idx++){
            //int core_id = m_platform_topo.domain_idx(GEOPM_DOMAIN_CORE, core_idx);
            //int pkg_id = m_platform_topo.domain_idx(GEOPM_DOMAIN_PACKAGE, core_idx);

            m_signal_idx[core_idx][M_SIGNAL_APERF] =
                m_platform_io.push_signal("MSR::APERF:ACNT"
                        ,GEOPM_DOMAIN_CORE
                        ,core_idx);
            m_signal_idx[core_idx][M_SIGNAL_MPERF] =
                m_platform_io.push_signal("MSR::MPERF:MCNT"
                        ,GEOPM_DOMAIN_CORE
                        ,core_idx);
            m_signal_idx[core_idx][M_SIGNAL_FREQ] = 
                m_platform_io.push_signal("CPU_FREQUENCY_STATUS"
                        ,GEOPM_DOMAIN_CORE
                        ,core_idx);

            m_signal_idx[core_idx][M_SIGNAL_PROGRESS] = 
                m_platform_io.push_signal("REGION_PROGRESS"
                        ,GEOPM_DOMAIN_CORE
                        ,core_idx);

        }
        for(int core_idx=0; core_idx < m_num_core; core_idx++){
            //m_control_idx[core_idx][M_CONTROL_FREQ] = 
            //    m_platform_io.push_control("CPU_FREQUENCY_CONTROL"
            //            ,GEOPM_DOMAIN_CORE
            //            ,core_idx);
            m_control_idx[core_idx][M_CONTROL_SETTING] = 
                m_platform_io.push_control("SST::COREPRIORITY:ASSOCIATION"
                        ,GEOPM_DOMAIN_CORE
                        ,core_idx);

        }

    }

}

