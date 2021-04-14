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

//TODO: Put doxygen in header

#ifndef CPUBALANCEAGENT_HPP_INCLUDE
#define CPUBALANCEAGENT_HPP_INCLUDE

#include <vector>

#include "Agent.hpp"
#include "geopm_time.h"
#include "TwoDMatrix.hpp"

namespace geopm
{
    class PlatformIO;
    class PlatformTopo;
    template <class type> class CircularBuffer;

    class CPUBalanceAgent : public Agent
    {
        public:
            CPUBalanceAgent();
            CPUBalanceAgent(PlatformIO &plat_io, const PlatformTopo &topo);
            virtual ~CPUBalanceAgent() = default;
            void init(int level, const std::vector<int> &fan_in, bool is_level_root) override;
            void validate_policy(std::vector<double> &policy) const override;
            void split_policy(const std::vector<double> &in_policy,
                              std::vector<std::vector<double> > &out_policy) override;
            bool do_send_policy(void) const override;
            void aggregate_sample(const std::vector<std::vector<double> > &in_sample,
                                  std::vector<double> &out_sample) override;
            bool do_send_sample(void) const override;
            void adjust_platform(const std::vector<double> &in_policy) override;
            bool do_write_batch(void) const override;
            void sample_platform(std::vector<double> &out_sample) override;
            void wait(void) override;
            std::vector<std::pair<std::string, std::string> > report_header(void) 
                const override;
            std::vector<std::pair<std::string, std::string> > report_host(void) 
                const override;
            std::map<uint64_t, std::vector<std::pair<std::string, std::string> > >
                report_region(void) const override;
            std::vector<std::string> trace_names(void) const override;
            std::vector<std::function<std::string(double)> > trace_formats(void)
                const override;
            void trace_values(std::vector<double> &values) override;
            void enforce_policy(const std::vector<double> &policy) const override;
            
            static std::string plugin_name(void);
            static std::unique_ptr<Agent> make_plugin(void);
            static std::vector<std::string> policy_names(void);
            static std::vector<std::string> sample_names(void);

        private:
            void init_platform_io(void);
            static bool is_all_nan(const std::vector<double> &vec);
            
            // Policy - send down from top
            //If you change this, update policy_names
            enum m_policy_e {
                M_POLICY_REBALANCE_ON,
                M_POLICY_FREQUENCY_MAX,
                M_POLICY_FREQUENCY_MIN,
                M_POLICY_HISTORY_WEIGHT,
                M_POLICY_CTRL_DELAY_ITER,
                M_POLICY_CTRL_RATE_SEC,
                M_NUM_POLICY
            };
            
            enum m_signal_e{
                //M_SIGNAL_PKG_POWER,
                M_SIGNAL_APERF,
                M_SIGNAL_MPERF,
                M_SIGNAL_FREQ,
                M_SIGNAL_PROGRESS,
                M_NUM_SIGNAL
            };

            enum m_ctrl_e{
                M_CONTROL_FREQ,
                M_NUM_CONTROL
            };

            PlatformIO &m_platform_io;
            const PlatformTopo &m_platform_topo;
            int m_level;
            int m_num_children;
            const double M_FREQ_STEP;
            const double M_WAIT_SEC;
            int m_buffer_size;
            int m_num_cpu;
            int m_num_core;
            int m_num_pkg;
            int m_num_iter;
            double m_core_freq_min;
            double m_core_freq_max;

            std::vector<std::vector<int> > m_signal_idx;
            std::vector<std::vector<int> > m_control_idx;
            std::vector<std::vector<double> > m_sample_new;
            std::vector<std::vector<double> > m_sample_hist;
            std::vector<double> m_freq;
            geopm_time_s m_last_wait;
            geopm_time_s m_last_balance;
            bool m_do_send_policy;
            bool m_do_write_batch;
            bool m_initialized_freq;
            std::vector<double> m_policy;
            std::vector<TwoDMatrix> m_freq_prog;
            double m_scale_factor;


            //Get region hash
            bool in_region();

            
            void set_all_cpu_freq(double freq_max);
           
            bool update_freq_range(const std::vector<double> &in_policy);

            //TODO: Add to TwoDMatrix the following:
            //      Takes in frequency, progress vectors
            //      Has saved state of freqprog vectors/matrices
            //      Outputs vector of desired frequencies 
            double progress_estimate();
            void distribute_freq_from_regression(double freq_min, double freq_max);
            bool skip_adjustment(int core_idx);
            void update_fit(double hist_weight);
           
            //These will go away
            void distribute_freq_linearly_ttc(double freq_min, double freq_max);
            void distribute_freq_linearly_prog(double freq_min, double freq_max);
            void distribute_freq_simply(double freq_min, double freq_max);
    };

}
#endif
