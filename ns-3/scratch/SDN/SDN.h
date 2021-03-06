/*
 * SDN.h
 *
 *  Created on: March 15, 2017
 *      Author: goals.lee
 */

#ifndef SDN_H
#define SDN_H

#include <string>
#include <fstream>
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/network-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/olsr-helper.h"
#include "ns3/aodv-helper.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"

//FINALLY!
#include "ns3/sdn-helper.h"

#include "ns3/vanetmobility-helper.h"

#include <unordered_set>
#include <unordered_map>
#include <vector>

using namespace ns3;

class VanetSim
{
public:
	VanetSim();
	~VanetSim();
	void Simulate(int argc, char *argv[]);
protected:
	void SetDefault();
	void ParseArguments(int argc, char *argv[]);
	void LoadTraffic();
	void ConfigNode();
	void ConfigChannels();
	void ConfigDevices();
	void ConfigMobility();
	void ConfigApp();
	void ConfigTracing();
	void Run();
	void ProcessOutputs();
	bool CheckActive(Node node);
	void Look_at_clock();
private:
	Ptr<Socket> source;
	std::string traceFile;
	std::string logFile;
	std::string phyMode;
	std::string lossModle;
	std::string homepath;
	std::string folder;
	std::ofstream os;
	double freq1;//SCH
	double freq2;//CCH
	double txp1;//SCH
	double txp2;//CCH
	double range1;//SCH
	double range2;//CCH
	uint32_t packetSize; // bytes
	uint32_t numPackets;
	double interval; // seconds
	bool verbose;
	int mod;//1=SDN Other=OLSR
	int pmod;//0=Range(Default) 1=Other
	uint32_t nodeNum;
	double duration;
	NodeContainer m_nodes;//Cars + 2Controller + Source + Sink
	NetDeviceContainer m_SCHDevices, m_CCHDevices;
	Ipv4InterfaceContainer m_SCHInterfaces, m_CCHInterfaces;
	//////////TongJi////////////
	uint32_t Rx_Routing_Bytes, Tx_Routing_Bytes;
	uint32_t RX_Routing_Pkts, TX_Routing_Pkts;
	uint32_t Rx1_Data_Bytes, Tx1_Data_Bytes,Rx2_Data_Bytes, Tx2_Data_Bytes;
	uint32_t Rx1_Data_Pkts, Tx1_Data_Pkts,Rx2_Data_Pkts, Tx2_Data_Pkts;
	uint32_t m_port;
	ApplicationContainer m_source1, m_sink1, m_cars, m_controller,m_source2, m_sink2;
	Ptr<ns3::vanetmobility::VANETmobility> VMo;
	void ReceiveDataPacket1 (Ptr<Socket> socket);
	void ReceiveDataPacket2 (Ptr<Socket> socket);
	void SendDataPacket ();
	void TXTrace1 (Ptr<const Packet> newpacket);
	void TXTrace2 (Ptr<const Packet> newpacket);

	std::unordered_map<uint64_t, Time> delay;
	std::vector<int64_t> delay_vector;
	std::unordered_set<uint64_t> dup_det;
	
	std::string m_todo;
  	std::string m_ds;//DataSet
  	int m_avg_forwardtimes;
  	int m_sum_messages;
};




#endif /* SDN_H */
