#include <stdio.h>
#include <NIDAQmx.h>
#include <chrono>
#include <thread>

#include "date.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "ni_daqmx_ros2/msg/waveform.hpp"



using namespace date;
using namespace std::chrono;

class DAQmxROS2 : public rclcpp::Node
{

public:
	DAQmxROS2()
		: Node("DAQmxROS2")
	{
		publisher_ = this->create_publisher<ni_daqmx_ros2::msg::Waveform>("daqmx_array", 10);
		timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DAQmxROS2::DAQmxCallback, this));

		DAQmxReserveNetworkDevice("cDAQ9189", 1); // Steal the cDAQ from anything connected to it

		DAQmxCreateTask("", &taskHandle);
		DAQmxCreateAIVoltageChan(taskHandle, "cDAQ9189Mod1/ai0:3", "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL);
		DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", sampleRate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, samplesN);
		time0 = std::chrono::system_clock::now();
		constexpr auto offset = sys_days{January / 1 / 1970} - sys_days{January / 1 / 1904};
		std::cout << "offset in seconds: " << seconds(offset) << std::endl;
		timestamp_s = std::chrono::duration_cast<std::chrono::seconds>(time0.time_since_epoch()).count();

		std::cout << "Current system time: " << timestamp_s << std::endl;
		std::cout << "CVIAbsoluteTime: " << timestamp_s + seconds(offset).count() << std::endl;

		int64_t startSecs = timestamp_s + seconds(offset).count() + startDelay; // wait 5 seconds before starting acquisition
		daqmx_timestamp_ns = (timestamp_s + startDelay) * 1e9;

		uInt64 fractionSec = 0; // start on the top of the second

		startTime.cviTime.msb = startSecs;	 // number of whole seconds after 12:00 a.m., Friday, January 1, 1904, Universal Time
		startTime.cviTime.lsb = fractionSec; // start on the second .00000

		returnINT = DAQmxCfgTimeStartTrig(taskHandle, startTime, DAQmx_Val_HostTime); //This ensures the precise t0 for the waveform
		std::cout << "return error 1: " << returnINT << std::endl;
		returnINT = DAQmxStartTask(taskHandle);
		std::cout << "return error 2: " << returnINT << std::endl;
		std::cout << "return error 3: " << returnINT << std::endl;
	}
	~DAQmxROS2()
	{
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}

private:
	int startDelay = 5; // seconds to wait before task is triggered
	float64 sampleRate = 50000.0; // Samples per second
	int samplesN = 5000; //samples per waveform
	float64 waveformPeriod = (float64)samplesN / sampleRate;
	uint64_t waveformPeriod_ns = waveformPeriod * 1e9;

	int32 error = 0;
	TaskHandle taskHandle = 0;
	char errBuff[2048] = {'\0'};
	int totalRead = 0;
	int32 read = 0;
	float64 data[20000];
	rclcpp::Publisher<ni_daqmx_ros2::msg::Waveform>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	ni_daqmx_ros2::msg::Waveform msg;
	std::vector<double> data_vector_ain0;
	std::vector<double> data_vector_ain1;
	std::vector<double> data_vector_ain2;
	std::vector<double> data_vector_ain3;
	int channel = 0;
	uint64_t timestamp_ns;
	uint64_t timestamp_s;
	uint64_t last_timestamp_ns;
	uint64_t daqmx_timestamp_ns;

	std::chrono::time_point<std::chrono::system_clock> time0;
	std::chrono::time_point<std::chrono::system_clock> time;
	CVIAbsoluteTime startTime;
	int32 returnINT;

	float64 *dataptr = data;
	void DAQmxCallback()
	{
		time = std::chrono::system_clock::now();
		returnINT = DAQmxReadAnalogF64(taskHandle, 5000, 10.0, DAQmx_Val_GroupByScanNumber, dataptr, 20000, &read, NULL); //read 1 waveform of 500 samps

		if (read > 0)
		{
			printf("Acquired %d samples. Total %d \t", (int)read, (int)(totalRead += read));
			printf("Data point 1 : %0.06f \t", data[19999]);

			for (channel = 0; channel < 20000 - 3; channel += 4)
			{

				data_vector_ain0.push_back(data[channel]);
				data_vector_ain1.push_back(data[channel + 1]);
				data_vector_ain2.push_back(data[channel + 2]);
				data_vector_ain3.push_back(data[channel + 3]);
			}

			timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
			std::cout << "size: " << data_vector_ain3.size() << "\t \t";
			std::cout << (double)timestamp_ns / 1000000000.0 - (double) daqmx_timestamp_ns / 1000000000.0 << "\r";

			//Put the DAQmx Waveform into a custom ROS message
			msg.set__ain0(data_vector_ain0);
			msg.set__ain1(data_vector_ain1);
			msg.set__ain2(data_vector_ain2);
			msg.set__ain3(data_vector_ain3);

			msg.set__daqmx_time(daqmx_timestamp_ns);
			msg.set__ros_time(timestamp_ns);
			publisher_->publish(msg);

			data_vector_ain0.clear();
			data_vector_ain1.clear();
			data_vector_ain2.clear();
			data_vector_ain3.clear();

			last_timestamp_ns = timestamp_ns;
			daqmx_timestamp_ns = daqmx_timestamp_ns + waveformPeriod_ns; //timestamp assumes cDAQ is sync'd with PTP
		}
	}
};
int main(int argc, char *argv[])
{

	rclcpp::init(argc, argv);
	auto node = std::make_shared<DAQmxROS2>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
