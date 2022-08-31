
//
// Created by yucedagonurcan on 2/17/21.
//

#ifndef TIMER_H
#define TIMER_H


#define TIMER_NAME_W_LABEL(label) std::string(__PRETTY_FUNCTION__) + " #" + std::to_string(__LINE__) + "_" + label
#define TIMER_NAME std::string(__PRETTY_FUNCTION__) + " #" + std::to_string(__LINE__)
#include "chrono"
#include <memory>

namespace Timer {

    struct TimerStatistics{
        double average_duration{0};
        double std_duration{0};
        int num_samples{0};
        double total_duration{0};
    };

    class Timer{
    public:
        Timer()= default;
        Timer(std::string label): m_label(std::move(label)) {}

        void PopulateStatistics(){
            ++stat.num_samples;
            stat.total_duration += current_duration;
            stat.average_duration = stat.total_duration / stat.num_samples;
        }

        void Start(){

            current_start_time = std::chrono::high_resolution_clock::now();

        }
        void Stop(){

            current_end_time = std::chrono::high_resolution_clock::now();
            current_duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_end_time - current_start_time).count();
            PopulateStatistics();
        }
        std::string GetLast(){
            std::string out_str = std::string("~ [ ") + m_label + std::string(" ] Time Difference: ") + std::to_string(current_duration) + std::string("[ms]");
            return out_str;
        }
        std::string GetStats(){
            std::stringstream out_str;
            out_str << "~ [ Statistics ] " <<  m_label  << "\n\t- Average Time: " << stat.average_duration << "[ms]" << "\n\t- Standard Dev: " << stat.std_duration << "[ms]";
            return out_str.str();
        }
        void PrintLast(){
            std::cout <<  GetLast() << std::endl;
        }

    protected:
        std::string m_label{};
        long current_duration{0};
        std::chrono::time_point<std::chrono::high_resolution_clock> current_start_time;
        std::chrono::time_point<std::chrono::high_resolution_clock> current_end_time;
        TimerStatistics stat;
    };
    class SeriesTimer  : public Timer
    {
    public:
        SeriesTimer()
        {
            current_start_time = std::chrono::high_resolution_clock::now();
        }
        explicit SeriesTimer(std::string label)
        {
            Timer::m_label = std::move(label);
            current_start_time = std::chrono::high_resolution_clock::now();
        }
    };
    class ScopedTimer : public Timer{
    public:
        ScopedTimer()
        {
            current_start_time = std::chrono::high_resolution_clock::now();
        }
        ScopedTimer(std::string label)
        {
            Timer::m_label = std::move(label);
            current_start_time = std::chrono::high_resolution_clock::now();
        }
        ~ScopedTimer(){
            Stop();
            PrintLast();
        }
    };
}

#define MakeTimer std::make_shared<Timer::SeriesTimer>
typedef std::shared_ptr<Timer::SeriesTimer> SeriesTimerPtr;
#endif //TIMER_H
