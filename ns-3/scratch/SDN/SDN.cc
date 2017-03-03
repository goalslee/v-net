/*
 * SDN.cc
 *
 *  Created on: Oct 9, 2015
 *      Author: chl
 */
/*
  ./waf --run "SDN"
*/
#include<stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <dirent.h>//DIR*
#include "SDN.h"

NS_LOG_COMPONENT_DEFINE ("SDN");


using namespace ns3;

VanetSim::VanetSim()
{
	traceFile = "";
	logFile = "SDN.log";
	phyMode = "OfdmRate6MbpsBW10MHz";
	lossModle = "ns3::FriisPropagationLossModel";
	freq1 = 5.860e9;  //802.11p SCH CH172
	freq2 = 5.890e9;  //802.11p CCH CH178
	txp1 = 20;  // dBm SCH
	txp2 = 20;  // CCH
	range1 = 400.0;//SCH
	range2 = 1000.0;//CCH
	//range2 = 700.0;//CCH
	packetSize = 1000; // bytes
	numPackets = 1;
	interval = 0.1; // seconds
	verbose = false;
	mod = 1;
	pmod = 0;
	duration = -1;
	nodeNum = 0;
	Rx_Data_Bytes = 0;
	Rx_Data_Pkts = 0;
	Rx_Routing_Bytes = 0;
	RX_Routing_Pkts = 0;
	Tx_Data_Bytes = 0;
	Tx_Data_Pkts = 0;
	Tx_Routing_Bytes = 0;
	TX_Routing_Pkts = 0;
	m_port = 65419;
	homepath = ".";//getenv("HOME");
	folder="SDNData";
}

VanetSim::~VanetSim()
{
	os.close();
}

void VanetSim::Simulate(int argc, char *argv[])
{
	SetDefault();
	ParseArguments(argc, argv);
	LoadTraffic();
	ConfigNode();
	ConfigChannels();
	ConfigDevices();
	ConfigMobility();
	ConfigApp();
	ConfigTracing();
	Run();
	ProcessOutputs();
	std::cout<<std::endl;
}

void VanetSim::SetDefault()
{
	//Handle By Constructor
}

void VanetSim::ParseArguments(int argc, char *argv[])
{
	CommandLine cmd;
//	cmd.AddValue ("traceFile", "Ns2 movement trace file", traceFile);
//	cmd.AddValue ("nodeNum", "Number of nodes", nodeNum);
	cmd.AddValue ("duration", "Duration of Simulation", duration);
//	cmd.AddValue ("logFile", "Log file", logFile);
	cmd.AddValue ("folder", "Working Directory", folder);
	cmd.AddValue ("txp1", "TX power for SCH", txp1);
	cmd.AddValue ("txp2", "TX power for CCH", txp2);
	cmd.AddValue ("range1", "Range for SCH", range1);
	cmd.AddValue ("range2", "Range for CCH", range2);
	cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
	cmd.AddValue ("mod", "0=olsr 1=sdn(DEFAULT) 2=aodv 3=dsdv 4=dsr", mod);
	cmd.AddValue ("pmod", "0=Range(DEFAULT) 1=Other", pmod);
	cmd.AddValue ("ds", "DataSet", m_ds);
	cmd.Parse (argc,argv);

	// Fix non-unicast data rate to be the same as that of unicast
	Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
	                      StringValue (phyMode));

}

void VanetSim::LoadTraffic()
{
	if (mod==0)
	{
		std::cout<<"Mode: OLSR-N"<<std::endl;
		m_todo = "OLSR";
	}
	else if(mod==1)
	{
		std::cout<<"Mode: SDN"<<std::endl;
		m_todo = "SDN";
	}
	else if(mod==2)
	{
		std::cout<<"Mode: AODV"<<std::endl;
		m_todo = "AODV";
	}
	else if(mod==3)
	{
		std::cout<<"Mode: DSR"<<std::endl;
		m_todo = "DSR";
	}
	else if(mod==4)
	{
		std::cout<<"Mode: DSDV"<<std::endl;
		m_todo = "DSDV";
	}
	DIR* dir = NULL;
	//DIR* subdir=NULL;
	std::string temp(homepath+"/"+folder);
	if((dir = opendir(temp.data()))==NULL)
		NS_FATAL_ERROR("Cannot open input path "<<temp.data()<<", Aborted.");

	//std::string sumo_net = temp + "/input.3net.xml";
	std::string sumo_net = temp + "/input.net.xml";

	//std::string sumo_fcd = temp + "/3fcd.xml";
	std::string sumo_fcd = temp + "/fcd.xml";
	std::string sumo_route = temp + "/rou.xml";
	//std::string sumo_route = temp + "/input.3rou.xml";

	std::string output = temp + "/" + m_todo + "_" + m_ds + "_result_new.txt";

	os.open(output.data(),std::ios::out);

	ns3::vanetmobility::VANETmobilityHelper mobilityHelper;
	VMo=mobilityHelper.GetSumoMObility(sumo_net,sumo_route,sumo_fcd);

	nodeNum = VMo->GetNodeSize();
	os<<"Mode:  "<<m_todo<<"DataSet:  "<<m_ds<<std::endl;
}



void VanetSim::ConfigNode()
{
	//m_nodes.Create(nodeNum+4);//Cars + 2Controller + Source + Sink
	m_nodes.Create(nodeNum+26);//Cars + 2Controller + Source + Sink ����nodemum+5����
	//std::cout<<nodeNum<<std::endl;
	/*Only Apps Are Different Between Different kind of Nodes*/
	// Name nodes
	for (uint32_t i = 0; i < nodeNum; ++i)
	{
		std::ostringstream os;
		os << "vehicle-" << i;
		Names::Add(os.str(), m_nodes.Get(i));//Ϊÿ�������
	}
	std::string str;
         char string[10];
	for(uint32_t i = 0; i < 24; ++i)//24 ��LC
	{
	   sprintf(string,"%d",i);
	   str=string;
	    Names::Add("Controller_"+str,m_nodes.Get(nodeNum+i));
	}

	Names::Add("Source",m_nodes.Get(nodeNum+24));//523
	Names::Add("Sink",m_nodes.Get(nodeNum+25));//524

}

void VanetSim::ConfigChannels()
{
	//===channel���������ŵ�,ͨ�ž����
	std::cout<<"ConfigChannels"<<std::endl;
	YansWifiChannelHelper SCHChannel;
	SCHChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	if (pmod == 1)
	{
		SCHChannel.AddPropagationLoss(lossModle,"Frequency", DoubleValue(freq1));
	}
	else
	{
		SCHChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange",
				DoubleValue(range1));
	}
	YansWifiChannelHelper CCHChannel;
	CCHChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	if (pmod ==1)
	{
		CCHChannel.AddPropagationLoss(lossModle,"Frequency", DoubleValue(freq2));
	}
	else
	{
		CCHChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange",
				DoubleValue(range2));
	}



	// the channelg
	Ptr<YansWifiChannel> SCH = SCHChannel.Create();
	Ptr<YansWifiChannel> CCH = CCHChannel.Create();

	//===wifiphy
	YansWifiPhyHelper SCHPhy =  YansWifiPhyHelper::Default ();
	SCHPhy.SetChannel (SCH);
	SCHPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
	YansWifiPhyHelper CCHPhy =  YansWifiPhyHelper::Default ();
	CCHPhy.SetChannel (CCH);
	CCHPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);

	// 802.11p mac
	NqosWaveMacHelper SCH80211pMac = NqosWaveMacHelper::Default ();
	Wifi80211pHelper SCH80211p = Wifi80211pHelper::Default ();
	NqosWaveMacHelper CCH80211pMac = NqosWaveMacHelper::Default ();
	Wifi80211pHelper CCH80211p = Wifi80211pHelper::Default ();

	SCH80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
										"DataMode",StringValue (phyMode),
										"ControlMode",StringValue (phyMode));
	CCH80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
											"DataMode",StringValue (phyMode),
											"ControlMode",StringValue (phyMode));

	// Set Tx Power For The SCH
	SCHPhy.Set ("TxPowerStart",DoubleValue (txp1));
	SCHPhy.Set ("TxPowerEnd", DoubleValue (txp1));
	m_SCHDevices = SCH80211p.Install(SCHPhy, SCH80211pMac, m_nodes);

	// CCH
	CCHPhy.Set ("TxPowerStart",DoubleValue (txp2));
	CCHPhy.Set ("TxPowerEnd", DoubleValue (txp2));
	m_CCHDevices = CCH80211p.Install(CCHPhy, CCH80211pMac, m_nodes);

}

void VanetSim::ConfigDevices()
{
	//Done in ConfigChannels()
}

void VanetSim::ConfigMobility()
{
/*	Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
	ns2.Install (m_nodes.Begin(),m_nodes.End()-3);
	// configure movements for Car node, while reading trace file
	Ptr<MobilityModel> Temp = m_nodes.Get(nodeNum);//Controller
	Temp->SetPosition(Vector(0.0, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+1);//source
	Temp->SetPosition(Vector(5.1, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+2);//Sink
*/
//���ÿ�������Դ��Ŀ�� ��λ��
	VMo->Install();
	double rt = VMo->GetReadTotalTime();
	if (duration<0)
	{
		duration = rt;
	}
	duration = 360;
	Time temp_now = Simulator::Now();
	std::cout<<"Now?"<<temp_now.GetSeconds ()<<std::endl;
	Ptr<MobilityModel> Temp = m_nodes.Get(nodeNum)->GetObject<MobilityModel>();//Controller1
	Temp->SetPosition(Vector(0.0, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+1)->GetObject<MobilityModel>();//source
	Temp->SetPosition(Vector(5.1, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+2)->GetObject<MobilityModel>();//Sink
	Temp->SetPosition(Vector(2000.0, 0.0, 0.0));
	//Temp->SetPosition(Vector(3000.0, 0.0, 0.0));
    Temp = m_nodes.Get(nodeNum+3)->GetObject<MobilityModel>();//Controller2
	//Temp->SetPosition(Vector(1000.0, 0.0, 0.0));
	Temp->SetPosition(Vector(700.0, 0.0, 0.0));
    Temp = m_nodes.Get(nodeNum+4)->GetObject<MobilityModel>();//Controller3
    //Temp->SetPosition(Vector(2000.0, 0.0, 0.0));
    Temp->SetPosition(Vector(1400.0, 0.0, 0.0));
}

void VanetSim::ConfigApp()
{
	//===����·���㷨Routing
	InternetStackHelper internet;
	if (mod == 0)
	{
		OlsrHelper olsr;
		//Ipv4ListRoutingHelper list;
		//list.Add(olsr,100);
		internet.SetRoutingHelper(olsr);
		std::cout<<"OLSR"<<std::endl;
		internet.Install (m_nodes);
	}
        else if (mod == 2)
	{
		AodvHelper aodv;
		internet.SetRoutingHelper(aodv);
		std::cout<<"AODV"<<std::endl;
		internet.Install (m_nodes);
	}
        else if (mod == 3)
	{
		DsrHelper dsr;
		//internet.SetRoutingHelper(dsr);
		DsrMainHelper dsrMain;
		std::cout<<"DSR"<<std::endl;
		internet.Install (m_nodes);
        	dsrMain.Install (dsr, m_nodes);
	}
        else if (mod == 4)
	{
		DsdvHelper dsdv;
		internet.SetRoutingHelper(dsdv);
		std::cout<<"DSDV"<<std::endl;
		internet.Install (m_nodes);
	}
	else
	{
	  SdnHelper sdn;
	  for (uint32_t i = 0; i<nodeNum; ++i)
	    {
	      sdn.SetNodeTypeMap (m_nodes.Get (i), sdn::CAR);
	    }
	  sdn.SetNodeTypeMap (m_nodes.Get (nodeNum), sdn::LOCAL_CONTROLLER);
          sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+3), sdn::LOCAL_CONTROLLER);
          sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+4), sdn::LOCAL_CONTROLLER);
	  sdn.ExcludeInterface (m_nodes.Get (nodeNum), 0);
	  sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+1), sdn::CAR);//Treat Source and Sink as CAR
	  sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+2), sdn::CAR);
	  sdn.SetRLnSR (range1, range2);//double signal_range, double road_length ���������?����wifiʱ�Ѿ����÷�Χ
	  internet.SetRoutingHelper(sdn);
		std::cout<<"SetRoutingHelper Done"<<std::endl;
	  internet.Install (m_nodes);
	}


	std::cout<<"internet.Install Done"<<std::endl;
	//===����IP ADDRESS

	Ipv4AddressHelper ipv4S;
	NS_LOG_INFO ("Assign IP Addresses.");
	ipv4S.SetBase ("10.1.1.0", "255.255.255.0");//SCH
	m_SCHInterfaces = ipv4S.Assign (m_SCHDevices);
	std::cout<<"IPV4S Assigned"<<std::endl;

	Ipv4AddressHelper ipv4C;
	if (mod ==1)
	{
		NS_LOG_INFO ("Assign IP-C Addresses.");
		ipv4C.SetBase("192.168.0.0","255.255.255.0");//CCH
		m_CCHInterfaces = ipv4C.Assign(m_CCHDevices);
		std::cout<<"IPV4C Assigned"<<std::endl;
		for (uint32_t i = 0;i<m_nodes.GetN ();++i)
		  {//��ÿ���ڵ��·��Э����������������ֵ
		    //std::cout<<"m_nodes.GetN () "<<i<<std::endl;
		    Ptr<sdn::RoutingProtocol> routing =
		        m_nodes.Get (i)->GetObject<sdn::RoutingProtocol> ();
        routing->SetCCHInterface (m_CCHInterfaces.Get (i).second);
		    routing->SetSCHInterface (m_SCHInterfaces.Get (i).second);
		  }
	}


	//===Traffic
	//source

	//onoff ����udp��
	/*std::pair<Ptr<Ipv4>, uint32_t> RetValue = m_SCHInterfaces.Get (nodeNum+1);
	Ipv4InterfaceAddress theinterface = RetValue.first->GetAddress (RetValue.second, 0);
  Ipv4Address bcast = theinterface.GetLocal ().GetSubnetDirectedBroadcast (theinterface.GetMask ());*/
  /*
	Address remote (InetSocketAddress(m_SCHInterfaces.GetAddress(nodeNum+2), m_port));
	OnOffHelper Source("ns3::UdpSocketFactory",remote);//SendToSink
	Source.SetAttribute("OffTime",StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
	DataRate x("4096bps");//512*8
	//Source.SetConstantRate(x,512);//
	Source.SetConstantRate(x,128);


	m_source = Source.Install(m_nodes.Get(nodeNum+1));//Install on Source
	m_source.Stop(Seconds(duration));//Default Start time is 0.
	std::string temp = "/NodeList/"+std::to_string (nodeNum+1)+"/ApplicationList/0/$ns3::OnOffApplication/Tx";

	Config::ConnectWithoutContext (
	    temp,
	    MakeCallback(&VanetSim::TXTrace, this));


	//sink
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Ptr<Socket> sink = Socket::CreateSocket (m_nodes.Get(nodeNum+2), tid);//The Sink
  //HearALL;
	//InetSocketAddress local = InetSocketAddress(m_CCHInterfaces.GetAddress(nodeNum+2),m_port);
	InetSocketAddress local = InetSocketAddress(Ipv4Address::GetZero (),m_port);
	sink->Bind(local);
	sink->SetRecvCallback(MakeCallback(&VanetSim::ReceiveDataPacket, this));
	*/
}

void VanetSim::ReceiveDataPacket(Ptr<Socket> socket)
{
	Ptr<Packet> packet;
	while ((packet = socket->Recv()))
	{
		Rx_Data_Bytes += packet->GetSize();
		Rx_Data_Pkts++;
		std::cout<<".";
	}
}

void VanetSim::SendDataPacket()
{
	/*Ptr<Packet> packet = Create<Packet> (packetSize);
	source->SendTo(packet, 0, )
	Simulator::Schedule(Seconds(interval), &VanetSim::SendDataPacket, this);*/
	//TODO
}

void VanetSim::ConfigTracing()
{
	//TODO
}

void VanetSim::ProcessOutputs()
{
	std::cout<<"send:"<<Tx_Data_Pkts<<std::endl;
	std::cout<<"recv:"<<Rx_Data_Pkts<<std::endl;

	os<<"Result:"<<std::endl;
  	os<<"Tx_Data_Pkts:   "<<Tx_Data_Pkts<<std::endl;
        os<<"Rx_Data_Pkts3:   "<<Rx_Data_Pkts<<std::endl;

}

void VanetSim::Run()
{
	Simulator::Schedule(Seconds(0.0), &VanetSim::Look_at_clock, this);
	std::cout << "Starting simulation for " << duration << " s ..."<< std::endl;
	os << "Starting simulation for " << duration << " s ..."<< std::endl;
	Simulator::Stop(Seconds(duration));
	Simulator::Run();
	Simulator::Destroy();

}

void VanetSim::Look_at_clock()
{
	std::cout<<"Now:"<<Simulator::Now().GetSeconds()<<std::endl;
	os<<"Now:  "<<Simulator::Now().GetSeconds()
  	<<"Tx_Data_Pkts:   "<<Tx_Data_Pkts
  	<<"Rx_Data_Pkts:   "<<Rx_Data_Pkts<<std::endl;
	/*Ptr<MobilityModel> Temp = m_nodes.Get (nodeNum)->GetObject<MobilityModel>();
  std::cout<<Temp->GetPosition().x<<","<<Temp->GetPosition().y<<","<<Temp->GetPosition().z<<std::endl;
  std::cout<<Temp->GetVelocity().x<<","<<Temp->GetVelocity().y<<","<<Temp->GetVelocity().z<<std::endl;
  Temp = m_nodes.Get (nodeNum+1)->GetObject<MobilityModel>();
  std::cout<<Temp->GetPosition().x<<","<<Temp->GetPosition().y<<","<<Temp->GetPosition().z<<std::endl;
  std::cout<<Temp->GetVelocity().x<<","<<Temp->GetVelocity().y<<","<<Temp->GetVelocity().z<<std::endl;
  Temp = m_nodes.Get (nodeNum+2)->GetObject<MobilityModel>();
  std::cout<<Temp->GetPosition().x<<","<<Temp->GetPosition().y<<","<<Temp->GetPosition().z<<std::endl;
  std::cout<<Temp->GetVelocity().x<<","<<Temp->GetVelocity().y<<","<<Temp->GetVelocity().z<<std::endl;
  */
	/*
	os<<"Now:"<<Simulator::Now().GetSeconds()<<std::endl;
	Ptr<OutputStreamWrapper> osw = Create<OutputStreamWrapper> (&std::cout);
	m_nodes.Get(nodeNum+1)->GetObject<Ipv4>()->GetRoutingProtocol()->PrintRoutingTable(osw);
	Ptr<OutputStreamWrapper> osw2 = Create<OutputStreamWrapper> (&os);
	m_nodes.Get(nodeNum+1)->GetObject<Ipv4>()->GetRoutingProtocol()->PrintRoutingTable(osw2);
	*/
	/*2  Ptr<MobilityModel> Temp;
	Vector vt;
	for (int i = 0;i<=nodeNum+2;++i)
	{
		Temp = m_nodes.Get(i)->GetObject<MobilityModel>();
		vt = Temp->GetPosition();
		std::cout<<i<<":"<<vt.x<<","<<vt.y<<","<<vt.z<<";"<<std::flush;
	}
	std::cout<<std::endl;*/
	//ProcessOutputs();

	Simulator::Schedule(Seconds(1.0), &VanetSim::Look_at_clock, this);
}

void
VanetSim::TXTrace (Ptr<const Packet> newpacket)
{
  Tx_Data_Pkts++;
  Tx_Data_Bytes += newpacket->GetSize ();
  //std::cout<<"ANOTHER ONE!HAHAHA"<<std::endl;
}

// Example to use ns2 traces file in ns3
int main (int argc, char *argv[])
{
         std::cout<<"begin";
	VanetSim SDN_test;
	SDN_test.Simulate(argc, argv);
	return 0;
}



