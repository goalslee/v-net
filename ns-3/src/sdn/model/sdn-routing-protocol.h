/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 Goals Lee
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Goals Lee <goals.lee@qq.com>
 */

#ifndef SDN_IMPL_H
#define SDN_IMPL_H

#include "sdn-header.h"

#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/node.h"
#include "ns3/socket.h"
#include "ns3/event-garbage-collector.h"
#include "ns3/random-variable-stream.h"
#include "ns3/timer.h"
#include "ns3/traced-callback.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/mobility-module.h"
//#include "sdn-duplicate-detection.h"

#include <vector>
#include <map>


namespace ns3 {
namespace sdn {


enum NodeType {CAR, LOCAL_CONTROLLER, OTHERS};
enum RoadType{ROW,COLUMN,NEITHER};
//enum direction{POSITIVE,NEGATIVE,OTHER};
/// An SDN's routing table entry.
struct RoutingTableEntry
{
  RoutingTableEntry () : // default values
                           destAddr (uint32_t(0)),
                           nextHop (uint32_t(0)),
                           mask (uint32_t(0)),
                           interface (0) {};

  Ipv4Address destAddr; ///< Address of the destination subnet.
  Ipv4Address nextHop; ///< Address of the next hop.
  Ipv4Address mask; ///< mask of the destination subnet.
  uint32_t interface; ///< Interface index. ָ���ĸ���������ȥ
};

// A struct for LC to hold Information that got from cars
class CarInfo
{
public:

  CarInfo () :
    Active (false)
  {
    dir=sdn::OTHER;
  };

  Vector3D Position;//λ��
  Vector3D Velocity;//�ٶ�
  Time LastActive;//Timeout indicator
  bool Active;
  std::vector<RoutingTableEntry> R_Table; //·�ɱ�
  direction dir;

  bool isTransfer;
};

struct AodvParm //
{

	uint32_t jumpnums;//����
	float stability;//�ȶ���
	Ipv4Address lastIP;//��һ��
	Ipv4Address nextIP;//��һ�����յ����ذ�ʱȷ��
	Ipv4Address m_sourceId;
          Ipv4Address m_desId;
          sdn::direction lastdir;//��һ���ķ�����һ���յ�֮��֪�����ĸ��������·
           Ipv4Address transfer;
};

struct AodvDesParm //
{

	uint32_t jumpnums;//����
	float stability;//�ȶ���
	sdn::direction desdir;//Ŀ�ĳ��ķ���
          bool dir;//Ŀ�ĳ��������������Ϣ�������Ƿ�һ��
	Ipv4Address lastIP;//��һ��
	Ipv4Address nextIP;//��һ�����յ����ذ�ʱȷ��
	Ipv4Address m_sourceId;
          Ipv4Address m_desId;
        sdn::direction lastdir;//��һ���ķ�����һ���յ�֮��֪�����ĸ��������·
};

class RoutingProtocol;

/// \brief SDN routing protocol for IPv4
///
class RoutingProtocol : public Ipv4RoutingProtocol
{

private:
  Ipv4Address m_SCHmainAddress;
  Ipv4Address m_CCHmainAddress;
  uint32_t m_SCHinterface;
  uint32_t m_CCHinterface;
  bool isDes=false;
  int m_firstRequest;
  std::map<Ipv4Address, Ipv4Address> m_SCHaddr2CCHaddr;
  Ipv4Address transferAddress;//now it is the nearest ip ÿ��·��һ����
  Ipv4Address roadendAddress;
  Ipv4Address transferAddress_possitive;//������ĵ�һ����
  Ipv4Address transferAddress_negative;//����������һ����

  Ipv4Address roadendAddress_possitive;//������ĵ�һ����
  Ipv4Address roadendAddress_negative;//����������һ����

  bool m_isEstablish_positive;
  bool m_isEstablish_negative;
  
  Ipv4Address temp_desId;
  //std::map<Ipv4Address, Ipv4Address> m_SCHaddr2IfaceAddr;
  // One socket per interface, each bound to that interface's address
  // (reason: for VANET-SDN we need to distinguish CCH and SCH interfaces)
  std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_socketAddresses;

  TracedCallback <const PacketHeader &,
                  const MessageList &> m_rxPacketTrace;   //����
  TracedCallback <const PacketHeader &,
                  const MessageList &> m_txPacketTrace;  //����
  TracedCallback <uint32_t> m_routingTableChanged;  //����

  /// Provides uniform random variables.
  Ptr<UniformRandomVariable> m_uniformRandomVariable;  

  std::map<Ipv4Address,std::set<Ipv4Address> >  neighbor;//ÿ��lc���ھ�lc

  // Mobility module for Vanet
  Ptr<MobilityModel> m_mobility;//�ڵ��λ�ú��ٶ���Ϣ
  std::set<uint32_t> m_interfaceExclusions;//����
  std::map<Ipv4Address, RoutingTableEntry> m_table; ///< Data structure for the routing table. (Use By Mainly by CAR Node, but LC needs it too) //m_table <Ŀ�ĵ�ַ��·����Ŀ>

  std::map<Ipv4Address, CarInfo> m_lc_info;///for LC
std::map<Ipv4Address, CarInfo> m_lc_positive_info;///for positive direction
std::map<Ipv4Address, CarInfo> m_lc_negative_info;///for negative direction


  std::vector<RoutingTableEntry> lc_table;

  //std::vector<RoutingTableEntry> lc_Rtable;//for aodv
  std::map<Ipv4Address,Ipv4Address> lc_Rtable;

  //std::map<Ipv4Address,AodvParm> m_AodvParm;//keep other lc's parm

  AodvParm m_selfParm_possitive{1000,1000};//lc'self parameter
  AodvParm m_selfParm_negative{1000,1000};
  AodvParm m_incomeParm_possitive{1000,1000};// received parameter
  AodvParm m_incomeParm_negative{1000,1000};
  AodvDesParm m_incomeDesParm{1000,1000,sdn::OTHER,true};
  //std::vector<Ipv4Address> m_ForwardTable;

  bool haveSource=false;
  bool haveSink=false;
  Ipv4Address m_sourceAddress;
  Ipv4Address m_sinkAddress;

  EventGarbageCollector m_events;
	
  /// Packets sequence number counter.
  uint16_t m_packetSequenceNumber;
  /// Messages sequence number counter.
  uint16_t m_messageSequenceNumber;

  double m_road_length;
  double m_signal_range;
  std::list<Ipv4Address> m_list4sort;


  bool possive_valid;
  bool negative_valid;

  uint32_t m_tag;

  /// HELLO messages' emission interval.
  Time m_helloInterval;
  /// Routing messages' emission interval.
  Time m_rmInterval;
  /// minimum ap message emission interval
  Time m_minAPInterval;

  Ptr<Ipv4> m_ipv4;
  NodeType m_nodetype;
  RoadType m_roadtype;
  //Only node type CAR use this(below)
  //AppointmentType m_appointmentResult;
  Ipv4Address m_next_forwarder;
  //bool m_linkEstablished;
  std::vector< std::set<Ipv4Address> > m_Sections;
  
  Ipv4Address m_theFirstCar;//Use by Reschedule (), SelectNewNodeInAreaZero(); Assign by SelectNode ();
  //Duplicate_Detection m_duplicate_detection;

typedef struct source_sink{
std::string source_sink; 
bool haveSource=false;
 Ipv4Address m_sourceAddress;
bool haveSink=false;
 Ipv4Address m_sinkAddress;
 bool isDes=false;
  AodvParm m_incomeParm_possitive{1000,1000};// received parameter
  AodvParm m_incomeParm_negative{1000,1000};
  AodvDesParm m_incomeDesParm{1000,1000,sdn::OTHER,true};
  uint32_t m_tag;
}ss_pair;

std::map<std::string,ss_pair> token; 
   public:
  static TypeId GetTypeId (void);//implemented

  RoutingProtocol ();//implemented
  virtual ~RoutingProtocol ();//implemented

  ///
  /// \brief Set the SDN main address to the first address on the indicated
  ///        interface
  /// \param interface IPv4 interface index
  ///
  void SetSCHInterface (uint32_t interface);//implemented ����SCHInterface������ֵm_SCHinterface�� SCH��ip  m_SCHmainAddress  ��ͬʱ��m_table�в���һ��Ŀ�ĵ�ַΪ�����·����Ϣ
  void SetCCHInterface (uint32_t interface);//implemented ͬ��
  
  void SetType (NodeType nt); //implemented //SdnHelper::Create �е���
  void SetRoadType (RoadType nt);
  NodeType GetType () const; //implemented  //��ȡ�ڵ����� CAR ���� CONTROLLER
  void SetSignalRangeNRoadLength (double signal_range, double road_length);// �������ã�SdnHelper::Create (Ptr<Node> node)ʱ����
  void SetMobility (Ptr<MobilityModel> mobility);//implemented //m_mobility ��ʼ������InternetStackHelper::Install (NodeContainer c)-��SdnHelper::Create (Ptr<Node> node)ʱ���� ��ͨ������ȡ�ٶȣ�λ�õ���ϢGetPosition() GetVelocity ()
    std::set<uint32_t> GetInterfaceExclusions () const //����
  {
    return (m_interfaceExclusions);
  }
  void SetInterfaceExclusions (std::set<uint32_t> exceptions);//implemented //����
  ///
  /// Dump the routing table
  /// to logging output (NS_LOG_DEBUG log level).  If logging is disabled,
  /// this function does nothing.
  ///
  void Dump (void);//implemented //����

  /**
   * Return the list of routing table entries discovered by SDN
   **/
  std::vector<RoutingTableEntry> GetRoutingTableEntries () const;//implemented //��m_table ��secondȫ��ȡ�����ŵ�vector��

 /**
  * Assign a fixed random variable stream number to the random variables
  * used by this model.  Return the number of streams (possibly zero) that
  * have been assigned.
  *
  * \param stream first stream index to use
  * \return the number of stream indices assigned by this model
  */
  int64_t AssignStreams (int64_t stream);//implemented //����



  
  private:
  
  void Clear ();//implemented ���m_table
  uint32_t GetSize () const { return (m_table.size ()); }
  void RemoveEntry (const Ipv4Address &dest);//implemented
  void AddEntry (const Ipv4Address &dest,
                 const Ipv4Address &mask,
                 const Ipv4Address &next,
                 uint32_t interface);//implemented
  void AddEntry (const Ipv4Address &dest,
                 const Ipv4Address &mask,
                 const Ipv4Address &next,
                 const Ipv4Address &interfaceAddress);//implemented
  bool Lookup (const Ipv4Address &dest,
               RoutingTableEntry &outEntry) const;//implemented

  // From Ipv4RoutingProtocol
  virtual Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p,
                                      const Ipv4Header &header,
                                      Ptr<NetDevice> oif,
                                      Socket::SocketErrno &sockerr);//implemented
  virtual bool RouteInput (Ptr<const Packet> p,
                           const Ipv4Header &header,
                           Ptr<const NetDevice> idev,
                           UnicastForwardCallback ucb,
                           MulticastForwardCallback mcb,
                           LocalDeliverCallback lcb,
                           ErrorCallback ecb);//implemented
  virtual void NotifyInterfaceUp (uint32_t interface);//implemented
  virtual void NotifyInterfaceDown (uint32_t interface);//implemented
  virtual void NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address);//implemented
  virtual void NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address);//implemented
  virtual void SetIpv4 (Ptr<Ipv4> ipv4);//implemented ��InternetStackHelper::Installʱ�����
  virtual void PrintRoutingTable (Ptr<OutputStreamWrapper> stream) const;//implemented

  void DoDispose ();//implemented�ر�socket�����m_table��m_ipv4�ȵ�ȫ������

  void SendPacket (Ptr<Packet> packet, const MessageList &containedMessages);//implemented �����message��Ϣ�İ���socket����ȥ

  /// Increments packet sequence number and returns the new value.
  inline uint16_t GetPacketSequenceNumber ();//implemented
  /// Increments message sequence number and returns the new value.
  inline uint16_t GetMessageSequenceNumber ();//implemented

  void RecvSDN (Ptr<Socket> socket);//implemented

  //Ipv4Address GetMainAddress (Ipv4Address iface_addr) const;

  // Timer handlers
  Timer m_helloTimer;
  void HelloTimerExpire ();//implemented

  Timer m_rmTimer;
  void RmTimerExpire ();//implemented

  Timer m_apTimer;
  void APTimerExpire ();

  Timer m_aodvTimer;
  void  AodvTimerExpire();

  Timer m_firstsendTimer;
  void FirstTimerExpire ();//implemented

  void sendfirstpackage();

  void Aodv_sendback(std::map<std::string,ss_pair>::iterator it);

  /// A list of pending messages which are buffered awaiting for being sent.
  sdn::MessageList m_queuedMessages;
  Timer m_queuedMessagesTimer; // timer for throttling outgoing messages

  void QueueMessage (const sdn::MessageHeader &message, Time delay);//implemented
  void SendQueuedMessages ();//implemented
  void SendHello ();//implemented
  void SendRoutingMessage (enum direction dir); //Fullfilled �����·�·�ɱ�

  void SendCRREQ(const Ipv4Address &destAddress);
  void SendCRREP(const Ipv4Address &sourceAddress,const Ipv4Address &destAddress,const Ipv4Address &transferAddress);

  void ProcessRm (const sdn::MessageHeader &msg);//implemented
  void ProcessHM (const sdn::MessageHeader &msg,const Ipv4Address &senderIface); //implemented
  void ProcessCRREQ (const sdn::MessageHeader &msg);
  void ProcessCRREP (Ipv4Address transfer,enum direction dir);
  void ProcessAodvRm(const sdn::MessageHeader &msg);
  void ProcessAodvRERm(const sdn::MessageHeader &msg);
  void ProcessMT(const sdn::MessageHeader &msg);
  void SetAodvParm(uint32_t jump,float sta);
  void GetAodvParm(uint32_t &jump,float &sta);
  void ComputeRoute ();//
  void compute_possive();
  void compute_negative();
  bool isNeighbor(const Ipv4Address &sourceAddress);
  void SendMT(enum direction dir,uint32_t n);

  /// Check that address is one of my interfaces
  bool IsMyOwnAddress (const Ipv4Address & a) const;//implemented

  void LCAddEntry( const Ipv4Address& ID,
                   const Ipv4Address& dest,
                   const Ipv4Address& mask,
                   const Ipv4Address& next);
  void LCAddEntry (const Ipv4Address& ID,
                   const Ipv4Address &dest,
                   const Ipv4Address &mask,
                   const Ipv4Address &next,
                   const Ipv4Address &interfaceAddress);
  void LCAddEntry (const Ipv4Address& ID,
                   const Ipv4Address &dest,
                   const Ipv4Address &mask,
                   const Ipv4Address &next,
                   uint32_t interface);
  void ClearAllTables ();


  void RemoveTimeOut ();


protected:
  virtual void DoInitialize (void);//implemented  ���ȱ�����  ����socket���ŵ������ü�����ʱ��
};


}
}  // namespace ns3

#endif /* SDN_IMPL_H */
