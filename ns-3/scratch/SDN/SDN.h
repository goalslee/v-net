/*
 * SDN.h
 *
 *  Created on: Oct 13, 2015
 *      Author: chl
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

//FINALLY!
#include "ns3/sdn-helper.h"

#include "ns3/vanetmobility-helper.h"


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
	NodeContainer m_nodes;//Cars + Controller + Source + Sink
	NetDeviceContainer m_SCHDevices, m_CCHDevices;
	Ipv4InterfaceContainer m_SCHInterfaces, m_CCHInterfaces;
	//////////TongJi////////////
	uint32_t Rx_Routing_Bytes, Tx_Routing_Bytes;
	uint32_t RX_Routing_Pkts, TX_Routing_Pkts;
	uint32_t Rx_Data_Bytes, Tx_Data_Bytes;
	uint32_t Rx_Data_Pkts, Tx_Data_Pkts;
	uint32_t m_port;
	ApplicationContainer m_source, m_sink, m_cars, m_controller;
	Ptr<ns3::vanetmobility::VANETmobility> VMo;
	void ReceiveDataPacket (Ptr<Socket> socket);
	void SendDataPacket ();
};




#endif /* SDN_H */
