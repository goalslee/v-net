/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 
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
 * 
 */


///
/// \brief Implementation of SDN agent on car side 
/// and related classes.
///
/// This is the main file of this software because SDN's behaviour is
/// implemented here.
///

#define NS_LOG_APPEND_CONTEXT                                   \
  if (GetObject<Node> ()) { std::clog << "[node " << GetObject<Node> ()->GetId () << "] "; }


#include "sdn-routing-protocol.h"
#include "ns3/socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/ipv4-route.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/ipv4-header.h"
#include <math.h>
#include <algorithm>

#include "stdlib.h" //ABS
#include "string.h"//memset
/********** Useful macros **********/

///
/// \brief Gets the delay between a given time and the current time.
///
/// If given time is previous to the current one, then this macro returns
/// a number close to 0. This is used for scheduling events at a certain moment.
///
#define DELAY(time) (((time) < (Simulator::Now ())) ? Seconds (0.000001) : \
                     (time - Simulator::Now () + Seconds (0.000001)))






/********** Miscellaneous constants **********/

/// Maximum allowed jitter.
#define SDN_MAXJITTER          (m_helloInterval.GetSeconds () / 4)
/// Random number between [0-SDN_MAXJITTER] used to jitter SDN packet transmission.
#define JITTER (Seconds (m_uniformRandomVariable->GetValue (0, SDN_MAXJITTER)))


#define SDN_MAX_SEQ_NUM        65535


#define SDN_PORT_NUMBER 419
/// Maximum number of messages per packet.
#define SDN_MAX_MSGS    64

#define ROAD_LENGTH 1000
#define SIGNAL_RANGE 400.0

#define INFHOP 2147483647

#define max_car_number 256
#define MAX 10000

namespace ns3 {
namespace sdn {

NS_LOG_COMPONENT_DEFINE ("SdnRoutingProtocol");


/********** SDN controller class **********/

NS_OBJECT_ENSURE_REGISTERED (RoutingProtocol);

TypeId
RoutingProtocol::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::sdn::RoutingProtocol")
    .SetParent<Ipv4RoutingProtocol> ()
    .AddConstructor<RoutingProtocol> ();
  return tid;
}


RoutingProtocol::RoutingProtocol ()
  :
    m_packetSequenceNumber (SDN_MAX_SEQ_NUM),
    m_messageSequenceNumber (SDN_MAX_SEQ_NUM),
    m_helloInterval (Seconds(2)),//no change!
    m_rmInterval (Seconds (2)),//no change!
    m_minAPInterval (Seconds (1)),//
    m_ipv4 (0),
    m_helloTimer (Timer::CANCEL_ON_DESTROY),
    m_rmTimer (Timer::CANCEL_ON_DESTROY),
    m_apTimer (Timer::CANCEL_ON_DESTROY),
    m_firstsendTimer(Timer::CANCEL_ON_DESTROY),
    m_queuedMessagesTimer (Timer::CANCEL_ON_DESTROY),
    m_SCHinterface (0),
    m_CCHinterface (0),
    m_nodetype (OTHERS),
    m_appointmentResult (NORMAL),
    m_next_forwarder (uint32_t (0)),
    m_linkEstablished (false),
    m_numArea (0),
    m_isPadding (false),
    m_numAreaVaild (false),
    m_road_length (1000),//MagicNumber
    m_signal_range (900),
    m_firstRequest(1),
    m_tag(0)
{
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();
}

RoutingProtocol::~RoutingProtocol ()
{
  
}

void
RoutingProtocol::SetIpv4 (Ptr<Ipv4> ipv4)
{
  NS_ASSERT (ipv4 != 0);
  NS_ASSERT (m_ipv4 == 0);
  NS_LOG_DEBUG ("Created sdn::RoutingProtocol");
  m_helloTimer.SetFunction 
    (&RoutingProtocol::HelloTimerExpire, this);
  m_queuedMessagesTimer.SetFunction 
    (&RoutingProtocol::SendQueuedMessages, this);
  m_rmTimer.SetFunction
    (&RoutingProtocol::RmTimerExpire, this);
  m_apTimer.SetFunction
    (&RoutingProtocol::APTimerExpire, this);
  /*m_firstsendTimer.SetFunction
    (&RoutingProtocol::FirstTimerExpire, this);
    */

  m_packetSequenceNumber = SDN_MAX_SEQ_NUM;
  m_messageSequenceNumber = SDN_MAX_SEQ_NUM;


  m_ipv4 = ipv4;
}

void RoutingProtocol::DoDispose ()
{
  m_ipv4 = 0;
//m_socketAddresses 鏄庡ぉ鐪嬬湅
  for (std::map< Ptr<Socket>, Ipv4InterfaceAddress >::iterator iter = 
       m_socketAddresses.begin ();
       iter != m_socketAddresses.end (); ++iter)
    {
      iter->first->Close ();
    }
  m_socketAddresses.clear ();
  m_table.clear();
  m_SCHaddr2CCHaddr.clear ();

  Ipv4RoutingProtocol::DoDispose ();
}

void
RoutingProtocol::PrintRoutingTable (Ptr<OutputStreamWrapper> stream) const
{
  std::ostream* os = stream->GetStream ();
  *os << "Destination\t\tMask\t\tNextHop\t\tInterface\tDistance\n";

  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator iter = 
       m_table.begin ();
       iter != m_table.end (); ++iter)
    {
      *os << iter->first << "\t\t";
      *os << iter->second.mask << "\t\t";
      *os << iter->second.nextHop << "\t\t";
      if (Names::FindName (m_ipv4->GetNetDevice (iter->second.interface)) != "")
        {
          *os << 
          Names::FindName (m_ipv4->GetNetDevice (iter->second.interface)) << 
          "\t\t";
        }
      else
        {
          *os << iter->second.interface << "\t\t";
        }
      *os << "\n";
    }
}
	
void 
RoutingProtocol::DoInitialize ()
{
  if (m_CCHmainAddress == Ipv4Address ()) //not run 因为m_CCHmainAddress在sdn.cc已经初始化
    {
     std::cout<<"test"<<std::endl;
      Ipv4Address loopback ("127.0.0.1");
      uint32_t count = 0;
      uint32_t count1 = 0;
      for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); ++i)
        {
          // CAR Use first address as ID
          // LC Use secend address as ID
          Ipv4Address addr = m_ipv4->GetAddress (i, 0).GetLocal ();
          if (addr != loopback)
            {
              if (m_nodetype == CAR)
                {
                  if(count1 == 1)
                      {
                        m_CCHmainAddress = addr;
                        break;
                      }
                  else if(count1 == 0)
                        m_SCHmainAddress = addr;
                  ++count1;
                  //std::cout<<i<<"123456 "<<addr.Get()<<std::endl;
                }
              else
                if (m_nodetype == LOCAL_CONTROLLER)
                  {
                    if (count == 1)
                      {
                        m_CCHmainAddress = addr;
                        break;
                      }
                    ++count;
                  }
            }
        }

      NS_ASSERT (m_CCHmainAddress != Ipv4Address ());
    } // if (m_CCHmainAddress == Ipv4Address ())

  NS_LOG_DEBUG ("Starting SDN on node " << m_CCHmainAddress);

  //设置每个lc的邻居  --硬编码
   //std::map<Ipv4Address,std::set<Ipv4Address>> neighbor
   std::set<Ipv4Address> temp;
   temp.insert(Ipv4Address ("192.168.2.2"));
   temp.insert(Ipv4Address ("192.168.2.3"));
   temp.insert(Ipv4Address ("192.168.1.247"));
   neighbor[Ipv4Address ("192.168.1.246")]=temp;

   temp.clear();
   temp.insert(Ipv4Address ("192.168.1.246"));
   temp.insert(Ipv4Address ("192.168.1.249"));
   temp.insert(Ipv4Address ("192.168.2.6"));
   neighbor[Ipv4Address ("192.168.2.2")]=temp;

    temp.clear();
    temp.insert(Ipv4Address ("192.168.1.246"));
    temp.insert(Ipv4Address ("192.168.1.249"));
    temp.insert(Ipv4Address ("192.168.2.7"));
    temp.insert(Ipv4Address ("192.168.1.250"));
    temp.insert(Ipv4Address ("192.168.1.247"));
    neighbor[Ipv4Address ("192.168.2.3")]=temp;
    
    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.2"));
    temp.insert(Ipv4Address ("192.168.2.3"));
    temp.insert(Ipv4Address ("192.168.2.7"));
    temp.insert(Ipv4Address ("192.168.2.6"));
    temp.insert(Ipv4Address ("192.168.1.250"));
    neighbor[Ipv4Address ("192.168.1.249")]=temp;

    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.4"));
    temp.insert(Ipv4Address ("192.168.2.3"));
    temp.insert(Ipv4Address ("192.168.2.7"));
    temp.insert(Ipv4Address ("192.168.2.8"));
    temp.insert(Ipv4Address ("192.168.1.249"));
    temp.insert(Ipv4Address ("192.168.1.251"));    
    neighbor[Ipv4Address ("192.168.1.250")]=temp;   

    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.4"));
    temp.insert(Ipv4Address ("192.168.2.3"));
    temp.insert(Ipv4Address ("192.168.1.246"));
    temp.insert(Ipv4Address ("192.168.1.248"));    
    neighbor[Ipv4Address ("192.168.1.247")]=temp; 

    temp.clear();
    temp.insert(Ipv4Address ("192.168.1.247"));
    temp.insert(Ipv4Address ("192.168.1.248"));
    temp.insert(Ipv4Address ("192.168.2.8"));
    temp.insert(Ipv4Address ("192.168.1.250"));
    temp.insert(Ipv4Address ("192.168.1.251"));
    neighbor[Ipv4Address ("192.168.2.4")]=temp;

    temp.clear();
    temp.insert(Ipv4Address ("192.168.1.250"));
    temp.insert(Ipv4Address ("192.168.2.9"));
    temp.insert(Ipv4Address ("192.168.2.8"));
    temp.insert(Ipv4Address ("192.168.2.4"));
    temp.insert(Ipv4Address ("192.168.2.5"));
    neighbor[Ipv4Address ("192.168.1.251")]=temp;    
    
    temp.clear();
    temp.insert(Ipv4Address ("192.168.1.251"));
    temp.insert(Ipv4Address ("192.168.1.248"));
    temp.insert(Ipv4Address ("192.168.2.9"));
    neighbor[Ipv4Address ("192.168.2.5")]=temp;

    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.4"));
    temp.insert(Ipv4Address ("192.168.2.5"));
    temp.insert(Ipv4Address ("192.168.1.247"));
    neighbor[Ipv4Address ("192.168.1.248")]=temp; 

    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.2"));
    temp.insert(Ipv4Address ("192.168.2.10"));
    temp.insert(Ipv4Address ("192.168.1.249"));
    temp.insert(Ipv4Address ("192.168.1.252"));    
    neighbor[Ipv4Address ("192.168.2.6")]=temp; 

    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.11"));
    temp.insert(Ipv4Address ("192.168.2.3"));
    temp.insert(Ipv4Address ("192.168.1.249"));
    temp.insert(Ipv4Address ("192.168.1.250")); 
    temp.insert(Ipv4Address ("192.168.1.252")); 
    temp.insert(Ipv4Address ("192.168.1.253")); 
    neighbor[Ipv4Address ("192.168.2.7")]=temp;     

    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.12"));
    temp.insert(Ipv4Address ("192.168.2.4"));
    temp.insert(Ipv4Address ("192.168.1.251"));
    temp.insert(Ipv4Address ("192.168.1.250")); 
    temp.insert(Ipv4Address ("192.168.1.254")); 
    temp.insert(Ipv4Address ("192.168.1.253")); 
    neighbor[Ipv4Address ("192.168.2.8")]=temp; 


    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.13"));
    temp.insert(Ipv4Address ("192.168.2.5"));
    temp.insert(Ipv4Address ("192.168.1.251")); 
    temp.insert(Ipv4Address ("192.168.1.254")); 
    neighbor[Ipv4Address ("192.168.2.9")]=temp; 

    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.13"));
    temp.insert(Ipv4Address ("192.168.2.9"));
    temp.insert(Ipv4Address ("192.168.2.8"));
    temp.insert(Ipv4Address ("192.168.2.12")); 
    temp.insert(Ipv4Address ("192.168.1.253")); 
    neighbor[Ipv4Address ("192.168.1.254")]=temp;

    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.11"));
    temp.insert(Ipv4Address ("192.168.2.8"));
    temp.insert(Ipv4Address ("192.168.2.7"));
    temp.insert(Ipv4Address ("192.168.2.12")); 
    temp.insert(Ipv4Address ("192.168.1.252")); 
    temp.insert(Ipv4Address ("192.168.1.254")); 
    neighbor[Ipv4Address ("192.168.1.253")]=temp; 

    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.11"));
    temp.insert(Ipv4Address ("192.168.2.8"));
    temp.insert(Ipv4Address ("192.168.2.7"));
    temp.insert(Ipv4Address ("192.168.2.12")); 
    temp.insert(Ipv4Address ("192.168.1.252")); 
    temp.insert(Ipv4Address ("192.168.1.254")); 
    neighbor[Ipv4Address ("192.168.1.252")]=temp;

    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.6"));
    temp.insert(Ipv4Address ("192.168.1.252")); 
    temp.insert(Ipv4Address ("192.168.1.255")); 
    neighbor[Ipv4Address ("192.168.2.10")]=temp;  
    
    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.7"));
    temp.insert(Ipv4Address ("192.168.2.0"));    
    temp.insert(Ipv4Address ("192.168.1.252")); 
    temp.insert(Ipv4Address ("192.168.1.253"));    
    temp.insert(Ipv4Address ("192.168.1.255")); 
    neighbor[Ipv4Address ("192.168.2.11")]=temp;    

    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.1"));
    temp.insert(Ipv4Address ("192.168.2.0"));   
    temp.insert(Ipv4Address ("192.168.2.8"));     
    temp.insert(Ipv4Address ("192.168.1.253"));    
    temp.insert(Ipv4Address ("192.168.1.254")); 
    neighbor[Ipv4Address ("192.168.2.12")]=temp;    

    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.1"));
    temp.insert(Ipv4Address ("192.168.2.9"));           
    temp.insert(Ipv4Address ("192.168.1.254")); 
    neighbor[Ipv4Address ("192.168.2.13")]=temp;

    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.10"));
    temp.insert(Ipv4Address ("192.168.2.0"));   
    temp.insert(Ipv4Address ("192.168.2.11"));     
    neighbor[Ipv4Address ("192.168.1.255")]=temp; 

    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.1"));
    temp.insert(Ipv4Address ("192.168.2.11"));   
    temp.insert(Ipv4Address ("192.168.2.12"));     
    temp.insert(Ipv4Address ("192.168.1.255"));    
    neighbor[Ipv4Address ("192.168.2.0")]=temp;   
    
    temp.clear();
    temp.insert(Ipv4Address ("192.168.2.0"));
    temp.insert(Ipv4Address ("192.168.2.13"));   
    temp.insert(Ipv4Address ("192.168.2.12"));        
    neighbor[Ipv4Address ("192.168.2.1")]=temp; 

  Ipv4Address loopback ("127.0.0.1");

  bool canRunSdn = false;
  //Install RecvSDN  Only on CCH channel.
  if(m_interfaceExclusions.find (m_CCHinterface) == m_interfaceExclusions.end ())//无用的判断
    {
      // Create a socket to listen only on this interface
      Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (),
                                                 UdpSocketFactory::GetTypeId ());
      // TRUE
      socket->SetAllowBroadcast (true);
      InetSocketAddress
        inetAddr (m_ipv4->GetAddress (m_CCHinterface, 0).GetLocal (), SDN_PORT_NUMBER);
      socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvSDN,  this));
      if (socket->Bind (inetAddr))
        {
          NS_FATAL_ERROR ("Failed to bind() SDN socket");
        }
      socket->BindToNetDevice (m_ipv4->GetNetDevice (m_CCHinterface)); //绑定之后所有包只通过该网卡发，从该网卡收到的包才会处理。
      //因为收到的是广播包，会从两张网卡进入socket
      m_socketAddresses[socket] = m_ipv4->GetAddress (m_CCHinterface, 0);

      canRunSdn = true;
    }

  Init_NumArea();
  if(canRunSdn)
    {
      HelloTimerExpire ();
      RmTimerExpire ();
      //APTimerExpire ();
      //FirstTimerExpire();
      NS_LOG_DEBUG ("SDN on node (Car) " << m_CCHmainAddress << " started");
    }
}

void 
RoutingProtocol::SetCCHInterface (uint32_t interface)
{
  //std::cout<<"SetCCHInterface "<<interface<<std::endl;
  m_CCHmainAddress = m_ipv4->GetAddress (interface, 0).GetLocal ();
  m_CCHinterface = interface;
  Ipv4InterfaceAddress temp_if_add = m_ipv4->GetAddress (m_CCHinterface, 0);
  AddEntry (temp_if_add.GetLocal (),
            Ipv4Address (temp_if_add.GetMask ().Get ()),
            temp_if_add.GetLocal (),
            m_CCHinterface);
  //std::cout<<"SetCCHInterface "<<m_CCHmainAddress.Get ()%256<<std::endl;
}

void 
RoutingProtocol::SetSCHInterface (uint32_t interface)
{
  //std::cout<<"SetSCHInterface "<<interface<<std::endl;
  m_SCHinterface = interface;
  m_SCHmainAddress = m_ipv4->GetAddress (interface, 0).GetLocal ();
  Ipv4InterfaceAddress temp_if_add = m_ipv4->GetAddress (m_SCHinterface, 0);
  AddEntry (temp_if_add.GetLocal (),
            Ipv4Address (temp_if_add.GetMask ().Get ()),
            temp_if_add.GetLocal (),
            m_SCHinterface);
  //std::cout<<"SetSCHInterface "<<m_SCHmainAddress.Get ()%256<<std::endl;
}

void
RoutingProtocol::SetInterfaceExclusions (std::set<uint32_t> exceptions)
{
  m_interfaceExclusions = exceptions;
}

//
// \brief Processes an incoming %SDN packet (Car Side).
void
RoutingProtocol::RecvSDN (Ptr<Socket> socket)
{
  //if (m_CCHmainAddress.Get () % 256 > 50)
 // std::cout<<"RecvSDN"<<m_CCHmainAddress.Get () % 256<<std::endl;
  Ptr<Packet> receivedPacket;
  Address sourceAddress;
  receivedPacket = socket->RecvFrom (sourceAddress);//CCH address

  InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom (sourceAddress);
  Ipv4Address senderIfaceAddr = inetSourceAddr.GetIpv4 ();
  Ipv4Address receiverIfaceAddr = m_socketAddresses[socket].GetLocal ();
  NS_ASSERT (receiverIfaceAddr != Ipv4Address ());
  NS_LOG_DEBUG ("SDN node " << m_CCHmainAddress
                << " received a SDN packet from "
                << senderIfaceAddr << " to " << receiverIfaceAddr);

                
  //std::cout<<"self: "<<m_CCHmainAddress<<"  income: "<<senderIfaceAddr<<" "<<isNeighbor(senderIfaceAddr)<<std::endl;
  
   bool isneighbor=isNeighbor(senderIfaceAddr);    
  // if(isneighbor) {std::cout<<"self: "<<m_CCHmainAddress<<"  income: "<<senderIfaceAddr<<" "<<isNeighbor(senderIfaceAddr)<<std::endl;}
  //std::cout<<"SDN node " << m_CCHmainAddress
               // << " received a SDN packet from "
                //<< senderIfaceAddr << " to " << receiverIfaceAddr<<std::endl;
  //std::cout<<"SDN node " << m_CCHmainAddress<<" received a SDN packet from "<<senderIfaceAddr<<" to "<<receiverIfaceAddr<<std::endl;
  // All routing messages are sent from and to port RT_PORT,
  // so we check it.
  NS_ASSERT (inetSourceAddr.GetPort () == SDN_PORT_NUMBER);

  Ptr<Packet> packet = receivedPacket;

  sdn::PacketHeader sdnPacketHeader;
  packet->RemoveHeader (sdnPacketHeader);
  NS_ASSERT (sdnPacketHeader.GetPacketLength () >= sdnPacketHeader.GetSerializedSize ());
  uint32_t sizeLeft = sdnPacketHeader.GetPacketLength () - sdnPacketHeader.GetSerializedSize ();

  MessageList messages;

  while (sizeLeft)
    {
      MessageHeader messageHeader;
      if (packet->RemoveHeader (messageHeader) == 0)
        NS_ASSERT (false);

      sizeLeft -= messageHeader.GetSerializedSize ();

      NS_LOG_DEBUG ("SDN Msg received with type "
                    << std::dec << int (messageHeader.GetMessageType ())
                    << " TTL=" << int (messageHeader.GetTimeToLive ())
                    << " SeqNum=" << messageHeader.GetMessageSequenceNumber ());
      /*std::cout<<"SDN Msg received with type "
                    << std::dec << int (messageHeader.GetMessageType ())
                    << " TTL=" << int (messageHeader.GetTimeToLive ())
                    << " SeqNum=" << messageHeader.GetMessageSequenceNumber ();*/
      messages.push_back (messageHeader);
    }

  m_rxPacketTrace (sdnPacketHeader, messages);
  
  for (MessageList::const_iterator messageIter = messages.begin ();
       messageIter != messages.end (); ++messageIter)
    {
      const MessageHeader &messageHeader = *messageIter;
      // If ttl is less than or equal to zero, or
      // the receiver is the same as the originator,
      // the message must be silently dropped
      //if ((messageHeader.GetTimeToLive () == 0)||(IsMyOwnAddress (sdnPacketHeader.originator)))
      if ((messageHeader.GetTimeToLive () == 0)||(messageHeader.GetOriginatorAddress () == m_CCHmainAddress))
        {
          // ignore it
          packet->RemoveAtStart (messageHeader.GetSerializedSize () - messageHeader.GetSerializedSize () );
          continue;
        }


      switch (messageHeader.GetMessageType ())
        {
        case sdn::MessageHeader::ROUTING_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s SDN node " << m_CCHmainAddress
                        << " received Routing message of size " 
                        << messageHeader.GetSerializedSize ());
          //Controller Node should discare Hello_Message
          if (GetType() == CAR)
            ProcessRm (messageHeader);
          break;

        case sdn::MessageHeader::HELLO_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s SDN node " << m_CCHmainAddress
                        << " received Routing message of size "
                        << messageHeader.GetSerializedSize ());
          //Car Node should discare Hello_Message
          if (GetType() == LOCAL_CONTROLLER)
          {
            ProcessHM (messageHeader,senderIfaceAddr);
          }
          break;

        /*case sdn::MessageHeader::APPOINTMENT_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s SDN node " << m_CCHmainAddress
                        << " received Appointment message of size "
                        << messageHeader.GetSerializedSize ());
          if (GetType() == CAR)
            ProcessAppointment (messageHeader);
          break;*/
        case sdn::MessageHeader::AODV_ROUTING_MESSAGE:  //add this for aodv
            NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                          << "s SDN node " << m_CCHmainAddress
                          << " received Aodv Routing message of size "
                          << messageHeader.GetSerializedSize ());
          if(GetType()==LOCAL_CONTROLLER&&isneighbor)
        	  ProcessAodvRm(messageHeader);
        	  break;
        case sdn::MessageHeader::AODV_REVERSE_MESSAGE:  //add this for aodv
            NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                          << "s SDN node " << m_CCHmainAddress
                          << " received Aodv Routing message of size "
                          << messageHeader.GetSerializedSize ());
            if(GetType()==LOCAL_CONTROLLER&&isneighbor)
                //std::cout<<"case"<<std::endl;
            	ProcessAodvRERm(messageHeader);
           break;
        case sdn::MessageHeader::CARROUTEREQUEST_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s SDN node " << m_CCHmainAddress
                        << " received CRREQ message of size "
                        << messageHeader.GetSerializedSize ());
             //std::cout<<"rreq"<<std::endl;           
          if (GetType() == LOCAL_CONTROLLER)
          {
             //std::cout<<"ProcessCRREQ "<<std::endl;
            ProcessCRREQ (messageHeader);
            }
          break;
        case sdn::MessageHeader::CARROUTERESPONCE_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s SDN node " << m_CCHmainAddress
                        << " received CRREP message of size "
                        << messageHeader.GetSerializedSize ());
          if (GetType() == LOCAL_CONTROLLER)
            //ProcessCRREP (messageHeader);
          break;
        default:
          NS_LOG_DEBUG ("SDN message type " <<
                        int (messageHeader.GetMessageType ()) <<
                        " not implemented");
        }

    }
    
}// End of RecvSDN

  bool
  RoutingProtocol::isNeighbor(const Ipv4Address &sourceAddress)
  {
    
    std::map<Ipv4Address,std::set<Ipv4Address>>::iterator  it=neighbor.find(m_CCHmainAddress);
    if(it==neighbor.end()) return false;
    std::set<Ipv4Address> ipset=it->second;
    if(ipset.find(sourceAddress)==ipset.end()) return false;
    return true;
  }


void
RoutingProtocol::ProcessHM (const sdn::MessageHeader &msg,const Ipv4Address &senderIface)
{

 //std::cout<<"ProcessHM roadtype: "<<m_roadtype<<std::endl;
  // 不在lc所属路的hello 包不收
  bool rev=false;
if(m_roadtype==sdn::ROW)
{
//std::cout<<"row"<<std::endl;
 if(msg.GetHello ().GetPosition ().x>=(m_mobility->GetPosition().x-500.0)&&msg.GetHello ().GetPosition ().x<=(m_mobility->GetPosition().x+500.0)
    &&msg.GetHello ().GetPosition ().y>=m_mobility->GetPosition().y&&msg.GetHello ().GetPosition ().y<=(m_mobility->GetPosition().y+20.0) )
    {
    rev=true;
    //std::cout<<"position "<<msg.GetHello ().GetPosition ().x<<std::endl;
    }
 }
 else if(m_roadtype==sdn::COLUMN)
 {
 //std::cout<<"column"<<std::endl;
  if(msg.GetHello ().GetPosition ().y>=(m_mobility->GetPosition().y-500.0)&&msg.GetHello ().GetPosition ().y<=(m_mobility->GetPosition().y+500.0)
    &&msg.GetHello ().GetPosition ().x>=m_mobility->GetPosition().x&&msg.GetHello ().GetPosition ().x<=(m_mobility->GetPosition().x+20.0) )
    rev=true;
 }

if(rev==false) return;

// std::cout<<"handle hellomessage"<<std::endl;
    Ipv4Address ID = msg.GetHello ().ID;//should be SCH address
  m_SCHaddr2CCHaddr[ID] = msg.GetOriginatorAddress();


  std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info.find (ID);

  if (it != m_lc_info.end ())
    {


      it->second.Active = true;
      it->second.LastActive = Simulator::Now ();
      it->second.Position = msg.GetHello ().GetPosition ();
      it->second.Velocity = msg.GetHello ().GetVelocity ();
      it->second.minhop = 0;
      

      std::map<Ipv4Address, CarInfo>::iterator iter = m_lc_positive_info.find (ID);
      std::map<Ipv4Address, CarInfo>::iterator iter2 = m_lc_negative_info.find (ID);      
      if(iter != m_lc_positive_info.end ())
      {
      iter->second.Active = true;
      iter->second.LastActive = Simulator::Now ();
      iter->second.Position = msg.GetHello ().GetPosition ();
      iter->second.Velocity = msg.GetHello ().GetVelocity ();
      iter->second.minhop = 0;        
      }
      else if(iter2 != m_lc_negative_info.end ())
      {
      iter2->second.Active = true;
      iter2->second.LastActive = Simulator::Now ();
      iter2->second.Position = msg.GetHello ().GetPosition ();
      iter2->second.Velocity = msg.GetHello ().GetVelocity ();
      iter2->second.minhop = 0;        
      }
      
    }
  else
    {
      CarInfo CI_temp;
      CI_temp.Active = true;
      CI_temp.LastActive = Simulator::Now ();
      CI_temp.Position = msg.GetHello ().GetPosition ();
      CI_temp.Velocity = msg.GetHello ().GetVelocity ();
      if(m_roadtype==sdn::ROW)
      {
            if(CI_temp.Velocity.x>=0.0)
            {
                CI_temp.dir=sdn::POSITIVE;
            }
            else {
                CI_temp.dir=sdn::NEGATIVE;
             }
      }
      else if(m_roadtype==sdn::COLUMN)
      {
        if(CI_temp.Velocity.y>=0.0) {CI_temp.dir=sdn::POSITIVE; //source && sink 都possive
  
        }
        else {CI_temp.dir=sdn::NEGATIVE;    
         
        }
      }
      m_lc_info[ID] = CI_temp;
      
      if(CI_temp.dir==sdn::POSITIVE) m_lc_positive_info[ID]=CI_temp;
      else m_lc_negative_info[ID]=CI_temp;
    }

    if(haveSource&&m_sourceAddress==ID)
    {
        std::map<Ipv4Address, CarInfo>::iterator it = m_lc_positive_info.find (ID);
        if(it!=m_lc_positive_info.end()) m_lc_positive_info.erase(it);
        
    }
        if(haveSink&&m_sinkAddress==ID)
    {
        std::map<Ipv4Address, CarInfo>::iterator it = m_lc_positive_info.find (ID);
        if(it!=m_lc_positive_info.end()) m_lc_positive_info.erase(it);
        
    }

    //std::cout<<"ip "<<m_CCHmainAddress;
    //std::cout<<" m_lc_positive_info "<<m_lc_positive_info.size()<<" m_lc_negative_info"<<m_lc_negative_info.size()<<std::endl;
}

// \brief Build routing table according to Rm
void
RoutingProtocol::ProcessRm (const sdn::MessageHeader &msg) //车收到lc发的路由表
{
  NS_LOG_FUNCTION (msg);
  
  const sdn::MessageHeader::Rm &rm = msg.GetRm();
  // Check if this rm is for me
  // Ignore rm that ID does not match.
  if (IsMyOwnAddress (rm.ID))
    {
      Time now = Simulator::Now();
      NS_LOG_DEBUG ("@" << now.GetSeconds() << ":Node " << m_CCHmainAddress
                    << "ProcessRm.");
      std::cout<<"Node " << m_CCHmainAddress<<std::endl;
      NS_ASSERT (rm.GetRoutingMessageSize() >= 0);

      Clear();

      SetCCHInterface(m_CCHinterface);
      SetSCHInterface(m_SCHinterface);
      //m_SCHaddr2CCHaddr.insert(std::map<Ipv4Address, Ipv4Address>::value_type(m_SCHmainAddress, m_CCHmainAddress));
      //m_SCHaddr2CCHaddr[m_SCHmainAddress] = m_CCHmainAddress;
      //std::cout<<"233 "<<m_SCHmainAddress.Get()<<" "<<m_CCHmainAddress.Get()<<std::endl;
      for (std::vector<sdn::MessageHeader::Rm::Routing_Tuple>::const_iterator it = rm.routingTables.begin();
            it != rm.routingTables.end();
            ++it)
      {

        AddEntry(it->destAddress,
                 it->mask,
                 it->nextHop,
                 m_SCHinterface);
      }
          if(m_CCHmainAddress==Ipv4Address("192.168.2.14"))
    {
        std::cout<<"match "<<m_CCHmainAddress<<std::endl;
        for(std::map<Ipv4Address, RoutingTableEntry>::iterator it= m_table.begin();it!=m_table.end();++it)
        {
            std::cout<<"route dest"<<it->second.destAddr<<" next"<<it->second.nextHop<<std::endl;
        }
    }
    }

}
 
/*void
RoutingProtocol::ProcessAppointment (const sdn::MessageHeader &msg)
{
  NS_LOG_FUNCTION (msg);
  const sdn::MessageHeader::Appointment &appointment = msg.GetAppointment ();
  if (IsMyOwnAddress (appointment.ID))
    {
      switch (appointment.ATField)
      {
        case NORMAL:
          //std::cout<<" \"NORMAL\""<<std::endl;
          break;
        case FORWARDER:
          m_next_forwarder = appointment.NextForwarder;
          //std::cout<<"CAR"<<m_CCHmainAddress.Get () % 256<<"ProcessAppointment";
          //std::cout<<" \"FORWARDER\""<<std::endl;
          //std::cout<<"NextForwarder:"<<m_next_forwarder.Get () % 256<<std::endl;
          break;
        default:
          std::cout<<" ERROR TYPE"<<std::endl;
      }
      m_appointmentResult = appointment.ATField;
    }
}
*/
void
RoutingProtocol::ProcessCRREQ (const sdn::MessageHeader &msg)
{
  NS_LOG_FUNCTION (msg);
  const sdn::MessageHeader::CRREQ &crreq = msg.GetCRREQ ();
  Ipv4Address dest =  crreq.destAddress;
  Ipv4Address source = crreq.sourceAddress;//the car's sch ip address


if(m_lc_info.find(source)==m_lc_info.end()) return;//the wrong lc get the packet
//std::cout<<"the wrong lc get the packet."<<std::endl;

  //add long road lc select
  if(m_lc_info.find(dest)==m_lc_info.end()){//forward to another LC ,connect to AODV routing
         //if(m_CCHmainAddress.Get()%256 == 84) return;//the last lc not have des,so just return;not for 84 to receive
              // if(m_firstRequest++>1) return;
               haveSource=true;
               m_sourceAddress=source;
               
                            std::map<Ipv4Address, CarInfo>::iterator it1 = m_lc_positive_info.find (m_sourceAddress);
                            std::map<Ipv4Address, CarInfo>::iterator it2 = m_lc_negative_info.find (m_sourceAddress);
                            if(it1!=m_lc_positive_info.end()) m_lc_positive_info.erase(it1);
                            if(it2!=m_lc_negative_info.end()) m_lc_negative_info.erase(it2);
             
               
               std::cout<<"handle request ,IP is "<<m_CCHmainAddress<<std::endl;
               ComputeRoute();
               if(!haveSource) std::cout<<"source not register"<<std::endl;
               if(m_lc_info[m_sourceAddress].dir==sdn::POSITIVE&&!possive_valid) {std::cout<<"no valid source route"<<std::endl;return;}
               else if(m_lc_info[m_sourceAddress].dir==sdn::NEGATIVE&&!negative_valid) {std::cout<<"no valid source route"<<std::endl;return;}
               //std::cout<<"computeroute finish"<<std::endl;
	     sdn::MessageHeader mesg;
		 //std::cout<<"forwarding..."<<std::endl;
		 mesg.SetMessageType(sdn::MessageHeader::AODV_ROUTING_MESSAGE);
		  Time now = Simulator::Now ();
		  mesg.SetVTime (m_helloInterval);
		  mesg.SetTimeToLive (8);
		  mesg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
		  sdn::MessageHeader::AodvRm &Aodvrm = mesg.GetAodvRm();
		  Aodvrm.tag=crreq.tag;
		  Aodvrm.ID=source;//
		  Aodvrm.DesId=dest;//
		  Aodvrm.mask=m_ipv4->GetAddress(0, 0).GetMask();

	
		  Aodvrm.jump_nums=m_selfParm_possitive.jumpnums;
		  Aodvrm.SetStability(m_selfParm_possitive.stability);
		  Aodvrm.Originator=m_CCHmainAddress;
		  Aodvrm.dir=sdn::POSITIVE;
		  Aodvrm.SetPosition(m_mobility->GetPosition().x, m_mobility->GetPosition(). y, m_mobility->GetPosition().z);

 
if(m_lc_info[m_sourceAddress].dir==sdn::POSITIVE){
		  m_incomeParm_possitive.m_desId=dest;
		  m_incomeParm_possitive.m_sourceId=source;
		  }
else if(m_lc_info[m_sourceAddress].dir==sdn::NEGATIVE){
		  m_incomeParm_negative.m_desId=dest;
		  m_incomeParm_negative.m_sourceId=source;
}

		  QueueMessage (mesg, JITTER);

  }
  else{//the first lc itself has des car
	
  }



}

void
RoutingProtocol::ProcessCRREP (Ipv4Address transfer,enum direction dir)
{
  //NS_LOG_FUNCTION (msg);
  /*const sdn::MessageHeader::CRREP &crrep = msg.GetCRREP ();
  Ipv4Address dest =  crrep.destAddress;
  Ipv4Address source = crrep.sourceAddress;
  Ipv4Address transfer = crrep.transferAddress;*/
  //const sdn::MessageHeader::Aodv_R_Rm &Aodv_r = msg.GetAodv_R_Rm();
  //Ipv4Address dest = Aodv_r.CarId;
  //Ipv4Address source =transferAddress;
  //Ipv4Address transfer = Aodv_r.ID;

 //std::cout<<"ProcessCRREP"<<transfer.Get()%256<<" "<<dest.Get()%256<<std::endl;

  Ipv4Address mask("255.255.0.0");
  //ComputeRoute();
  //LCAddEntry(roadendAddress,dest,mask,transfer);
  //std::cout<<"roadendAddress"<<roadendAddress.Get()%256<<std::endl;
 // std::cout<<"infosize"<<m_lc_info.size()<<std::endl;
  //std::cout<<"roadendAddress"<<roadendAddress.Get()%256<<std::endl;

  for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
       cit != m_lc_info.end (); ++cit)
    {

          if(dir==sdn::POSITIVE){
          if(cit->second.dir==sdn::POSITIVE){
	 CarInfo Entry = cit->second;

	  for(std::vector<RoutingTableEntry>::iterator it=Entry.R_Table.begin();it!=Entry.R_Table.end();++it)
	  {

	      if(it->destAddr == roadendAddress_possitive)
	      {
	          if(cit->first!=roadendAddress_possitive){

	    	  LCAddEntry(cit->first,m_incomeParm_possitive.m_desId,it->mask,it->nextHop);
	
	    	  }
	    	  else{
	    	     LCAddEntry(cit->first,m_incomeParm_possitive.m_desId,it->mask,transfer);
	    	  }
	      }
	  }
	  }
	  }
	  else{
	            if(cit->second.dir==sdn::NEGATIVE){
	  CarInfo Entry = cit->second;
	
	  for(std::vector<RoutingTableEntry>::iterator it=Entry.R_Table.begin();it!=Entry.R_Table.end();++it)
	  {

	      if(it->destAddr == roadendAddress_negative)
	      {
	          if(cit->first!=roadendAddress_negative){

	    	  LCAddEntry(cit->first,m_incomeParm_negative.m_desId,it->mask,it->nextHop);
	
	    	  }
	    	  else{
	    	     LCAddEntry(cit->first,m_incomeParm_negative.m_desId,it->mask,transfer);
	    	  }
	      }
	  }	  
	  }
	  }

    }

  	 SendRoutingMessage(dir);
}

void
RoutingProtocol::Clear()
{
  NS_LOG_FUNCTION_NOARGS();
  m_table.clear();
}

void
RoutingProtocol::AddEntry (const Ipv4Address &dest,
                           const Ipv4Address &mask,
                           const Ipv4Address &next,
                           uint32_t interface)
{
  NS_LOG_FUNCTION(this << dest << next << interface << mask << m_CCHmainAddress);
  //std::cout<<"dest:"<<m_next_forwarder.Get () % 256<<std::endl;
  RoutingTableEntry RTE;
  RTE.destAddr = dest;
  RTE.mask = mask;
  RTE.nextHop = next;
  RTE.interface = interface;
  m_table[dest] = RTE;
}

void
RoutingProtocol::AddEntry (const Ipv4Address &dest,
                           const Ipv4Address &mask,
                           const Ipv4Address &next,
                           const Ipv4Address &interfaceAddress)
{
  NS_LOG_FUNCTION(this << dest << next << interfaceAddress << mask << m_CCHmainAddress);

  NS_ASSERT (m_ipv4);

  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces(); ++i)
   for (uint32_t j = 0; j< m_ipv4->GetNAddresses(i); ++j)
     {
       if (m_ipv4->GetAddress(i,j).GetLocal() == interfaceAddress)
         {
           AddEntry(dest, mask, next, i);
           return;
         }
     }
  NS_ASSERT(false);
  //AddEntry(dest, mask, next, 0);
}

bool
RoutingProtocol::Lookup(Ipv4Address const &dest,
                        RoutingTableEntry &outEntry) const
{
  std::map<Ipv4Address, RoutingTableEntry>::const_iterator it =
    m_table.find(dest);
  if (it != m_table.end())
    {
      outEntry = it->second;
      return true;
    }
  else
    {
        return false;
    }

}

void
RoutingProtocol::RemoveEntry (Ipv4Address const &dest)
{
  m_table.erase (dest);
}


bool
RoutingProtocol::RouteInput(Ptr<const Packet> p,
                            const Ipv4Header &header,
                            Ptr<const NetDevice> idev,
                            UnicastForwardCallback ucb,//正常传递消息需要
                            MulticastForwardCallback mcb,
                            LocalDeliverCallback lcb,//找路和找lc时发的广播包，所以会往上传递
                            ErrorCallback ecb)
{
  NS_LOG_FUNCTION (this << " " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination ());
  //TODO
  //std::cout<<m_mainAddress.Get ()%256<<" "<<header.GetDestination ().Get () %256<<std::endl;
  //bool lcb_status = false;
  Ipv4Address dest = header.GetDestination();
  Ipv4Address sour = header.GetSource();

  // Consume self-originated packets
  if (IsMyOwnAddress (sour) == true)
    {
      return true;
    }

  // Local delivery
  NS_ASSERT (m_ipv4->GetInterfaceForDevice (idev) >= 0);
  uint32_t iif = m_ipv4->GetInterfaceForDevice (idev);//SCH dev!
  if (m_ipv4->IsDestinationAddress (dest, iif))
    {
      //Duplicate Detection. TODO
      //if (m_duplicate_detection.CheckThis ())
      //Local delivery
      if (!lcb.IsNull ())
        {
          NS_LOG_LOGIC ("Broadcast local delivery to " << dest);
          //std::cout<<"local delivery"<<std::endl;
          lcb (p, header, iif);
          return true;
        }
      else
        {
          NS_LOG_ERROR ("Unable to deliver packet locally due to null callback");
          ecb (p, header, Socket::ERROR_NOROUTETOHOST);
          return false;
        }
   }
      /*//Broadcast forward
      if ((iif == m_SCHinterface) && (m_nodetype == CAR) && (m_appointmentResult == FORWARDER) && (sour != m_next_forwarder))
        {
          NS_LOG_LOGIC ("Forward broadcast");
          Ptr<Ipv4Route> broadcastRoute = Create<Ipv4Route> ();
          broadcastRoute->SetDestination (dest);
          broadcastRoute->SetGateway (dest);//broadcast
          broadcastRoute->SetOutputDevice (m_ipv4->GetNetDevice (m_SCHinterface));
          broadcastRoute->SetSource (sour);
          //std::cout<<"call ucb"<<std::endl;
          ucb (broadcastRoute, p, header);
        }*/
      
      //Forwardding
      Ptr<Ipv4Route> rtentry;
      RoutingTableEntry entry;
      std::cout<<"RouteInput "<<m_SCHmainAddress<< ",Dest:"<<header.GetDestination ()<<std::endl;
      //std::cout<<"M_TABLE SIZE "<<m_table.size ()<<std::endl;
      if (Lookup (header.GetDestination (), entry))
      {
          std::cout<<"found!nextHop "<<entry.nextHop<<std::endl;
          uint32_t interfaceIdx = entry.interface;
          rtentry = Create<Ipv4Route> ();
          rtentry->SetDestination (header.GetDestination ());
          // the source address is the interface address that matches
          // the destination address (when multiple are present on the
          // outgoing interface, one is selected via scoping rules)
          NS_ASSERT (m_ipv4);
          uint32_t numOifAddresses = m_ipv4->GetNAddresses (interfaceIdx);
          NS_ASSERT (numOifAddresses > 0);
          Ipv4InterfaceAddress ifAddr;
          if (numOifAddresses == 1) {
              ifAddr = m_ipv4->GetAddress (interfaceIdx, 0);
          } else {
              NS_FATAL_ERROR ("XXX Not implemented yet:  IP aliasing and SDN");
          }
          rtentry->SetSource (ifAddr.GetLocal ());
          rtentry->SetGateway (entry.nextHop);
          rtentry->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));
          NS_LOG_DEBUG ("SDN node " << m_CCHmainAddress
                                 << ": RouteInput for dest=" << header.GetDestination ()
                                 << " --> nextHop=" << entry.nextHop
                                 << " interface=" << entry.interface);
          NS_LOG_DEBUG ("Found route to " << rtentry->GetDestination () << " via nh " << rtentry->GetGateway () << " with source addr " << rtentry->GetSource () << " and output dev " << rtentry->GetOutputDevice ());
          ucb (rtentry, p, header);
      }
      else
      {
          NS_LOG_DEBUG ("SDN node " << m_CCHmainAddress
                                 << ": RouteInput for dest=" << header.GetDestination ()
                                 << " No route to host");
          //std::cout<<"2No route to host"<<std::endl;
      }
        
      return true;

    /*}
  //Drop
  return true;*/
}

void
RoutingProtocol::NotifyInterfaceUp (uint32_t i)
{}
void
RoutingProtocol::NotifyInterfaceDown (uint32_t i)
{}
void
RoutingProtocol::NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address)
{}
void
RoutingProtocol::NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address)
{}

Ptr<Ipv4Route>
RoutingProtocol::RouteOutput (Ptr<Packet> p,
             const Ipv4Header &header,
             Ptr<NetDevice> oif,
             Socket::SocketErrno &sockerr)
{
 //std::cout<<"RouteOutput "<<std::endl;
  NS_LOG_FUNCTION (this << " " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination () << " " << oif);
  Ptr<Ipv4Route> rtentry;
  RoutingTableEntry entry;
  //std::cout<<"RouteOutput "<<m_SCHmainAddress.Get () << ",Dest:"<<header.GetDestination ().Get ()<<std::endl;
  //std::cout<<"M_TABLE SIZE "<<m_table.size ()<<std::endl;
  if (Lookup (header.GetDestination (), entry))//如果是广播，则m_table须有广播地址条目
    {
      uint32_t interfaceIdx = entry.interface;
      if (oif && m_ipv4->GetInterfaceForDevice (oif) != static_cast<int> (interfaceIdx))
        {
          // We do not attempt to perform a constrained routing search
          // if the caller specifies the oif; we just enforce that
          // that the found route matches the requested outbound interface
          NS_LOG_DEBUG ("SDN node " << m_SCHmainAddress
                                     << ": RouteOutput for dest=" << header.GetDestination ()
                                     << " Route interface " << interfaceIdx
                                     << " does not match requested output interface "
                                     << m_ipv4->GetInterfaceForDevice (oif));
          sockerr = Socket::ERROR_NOROUTETOHOST;
          std::cout<<"does not match requested output interface"<<std::endl;
          return rtentry;
        }
      rtentry = Create<Ipv4Route> ();
      rtentry->SetDestination (header.GetDestination ());
      // the source address is the interface address that matches
      // the destination address (when multiple are present on the
      // outgoing interface, one is selected via scoping rules)
      NS_ASSERT (m_ipv4);
      uint32_t numOifAddresses = m_ipv4->GetNAddresses (interfaceIdx);
      NS_ASSERT (numOifAddresses > 0);
      Ipv4InterfaceAddress ifAddr;
      if (numOifAddresses == 1) {
          ifAddr = m_ipv4->GetAddress (interfaceIdx, 0);
        } else {
          NS_FATAL_ERROR ("XXX Not implemented yet:  IP aliasing and SDN");
        }
      rtentry->SetSource (ifAddr.GetLocal ());
      rtentry->SetGateway (entry.nextHop);
      rtentry->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));
      sockerr = Socket::ERROR_NOTERROR;
      std::cout<<"***"<<rtentry->GetDestination ()<<" "<<rtentry->GetGateway ()<<std::endl;
      NS_LOG_DEBUG ("SDN node " << m_SCHmainAddress
                                 << ": RouteOutput for dest=" << header.GetDestination ()
                                 << " --> nextHop=" << entry.nextHop
                                 << " interface=" << entry.interface);
      NS_LOG_DEBUG ("Found route to " << rtentry->GetDestination () << " via nh " << rtentry->GetGateway () << " with source addr " << rtentry->GetSource () << " and output dev " << rtentry->GetOutputDevice ());
    }
  else
    {
      NS_LOG_DEBUG ("SDN node " << m_SCHmainAddress
                                 << ": RouteOutput for dest=" << header.GetDestination ()
                                 << " No route to host");
      sockerr = Socket::ERROR_NOROUTETOHOST;
      SendCRREQ(header.GetDestination());
      //std::cout<<"dest "<<header.GetDestination()<<std::endl;
      //std::cout<<"No route to host"<<std::endl;
    }
  return rtentry;
}

void
RoutingProtocol::Dump ()
{
#ifdef NS3_LOG_ENABLE
  NS_LOG_DEBUG ("Dumpping For" << m_SCHmainAddress);
#endif //NS3_LOG_ENABLE
}

std::vector<RoutingTableEntry>
RoutingProtocol::GetRoutingTableEntries () const
{
  std::vector<RoutingTableEntry> rtvt;
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator it = m_table.begin ();
       it != m_table.end (); ++it)
    {
      rtvt.push_back (it->second);
    }
  return rtvt;
}

int64_t
RoutingProtocol::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uniformRandomVariable->SetStream (stream);
  return 1;
}

uint16_t
RoutingProtocol::GetPacketSequenceNumber ()
{
  m_packetSequenceNumber = (m_packetSequenceNumber + 1) % (SDN_MAX_SEQ_NUM + 1);
  return m_packetSequenceNumber;
}


uint16_t
RoutingProtocol::GetMessageSequenceNumber ()
{
  m_messageSequenceNumber = (m_messageSequenceNumber + 1) % (SDN_MAX_SEQ_NUM + 1);
  return m_messageSequenceNumber;
}

void
RoutingProtocol::HelloTimerExpire ()
{
  //std::cout<<"HmTimerExpire "<<m_CCHmainAddress.Get ()%256;
  //std::cout<<", Time:"<<Simulator::Now().GetSeconds ()<<std::endl;
  if (GetType() == CAR)
    {
      SendHello ();
      m_helloTimer.Schedule (m_helloInterval);
    }
}

void
RoutingProtocol::RmTimerExpire ()
{
  //Do nothing.
 // std::cout<<"RmTimerExpire "<<m_CCHmainAddress.Get ()%256;
  //std::cout<<", Time:"<<Simulator::Now().GetSeconds ()<<std::endl;

  if (GetType () == LOCAL_CONTROLLER)
  {
     // ClearAllTables();//std::cout<<"1:"<<std::endl;
      ComputeRoute ();//std::cout<<"2:"<<std::endl;
      //SendRoutingMessage ();//std::cout<<"3:"<<std::endl;
      m_rmTimer.Schedule (m_rmInterval);//std::cout<<"4:"<<std::endl;
  }
}

void
RoutingProtocol::APTimerExpire ()
{
  /*if (GetType() == LOCAL_CONTROLLER)
    {
      ComputeRoute ();
    }*/
}


// SDN packets actually send here.
void
RoutingProtocol::SendPacket (Ptr<Packet> packet,
                             const MessageList &containedMessages)
{
  NS_LOG_DEBUG ("SDN node " << m_CCHmainAddress << " sending a SDN packet");
  //std::cout<<"SDN node " << m_CCHmainAddress.Get ()<< " sending a SDN packet"<<std::endl;
  // Add a header
  sdn::PacketHeader header;
  header.originator = this->m_CCHmainAddress;
  header.SetPacketLength (header.GetSerializedSize () + packet->GetSize ());
  header.SetPacketSequenceNumber (GetPacketSequenceNumber ());
  packet->AddHeader (header);

  // Trace it
  m_txPacketTrace (header, containedMessages);

  // Send it
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator i =
         m_socketAddresses.begin (); i != m_socketAddresses.end (); ++i)
    {
      Ipv4Address bcast = i->second.GetLocal ().GetSubnetDirectedBroadcast (i->second.GetMask ());
      i->first->SendTo (packet, 0, InetSocketAddress (bcast, SDN_PORT_NUMBER));
    }
}

void
RoutingProtocol::QueueMessage (const sdn::MessageHeader &message, Time delay)
{
   m_queuedMessages.push_back (message);
  if (not m_queuedMessagesTimer.IsRunning ())
    {
      m_queuedMessagesTimer.SetDelay (delay);
      m_queuedMessagesTimer.Schedule ();
    }
}


// NS3 is not multithread, so mutex is unnecessary.
// Here, messages will queue up and send once numMessage is equl to SDN_MAX_MSGS.
// This function will NOT add a header to each message
void
RoutingProtocol::SendQueuedMessages ()
{
  Ptr<Packet> packet = Create<Packet> ();
  int numMessages = 0;

  NS_LOG_DEBUG ("SDN node " << m_CCHmainAddress << ": SendQueuedMessages");
  //std::cout<<"SendQueuedMessages  "<<m_CCHmainAddress.Get ()%256 <<std::endl;
  MessageList msglist;

  for (std::vector<sdn::MessageHeader>::const_iterator message = m_queuedMessages.begin ();
       message != m_queuedMessages.end ();
       ++message)
    {
      Ptr<Packet> p = Create<Packet> ();
      p->AddHeader (*message);
      packet->AddAtEnd (p);
      msglist.push_back (*message);
      if (++numMessages == SDN_MAX_MSGS)
        {
          SendPacket (packet, msglist);
          msglist.clear ();
          // Reset variables for next packet
          numMessages = 0;
          packet = Create<Packet> ();
        }
    }

  if (packet->GetSize ())
    {
      SendPacket (packet, msglist);
    }

  m_queuedMessages.clear ();
}

bool
RoutingProtocol::IsMyOwnAddress (const Ipv4Address & a) const
{
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
         m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
    {
      Ipv4InterfaceAddress iface = j->second;
      if (a == iface.GetLocal ())
        {
          return true;
        }
    }
  return false;
}

void
RoutingProtocol::SendHello ()
{
  NS_LOG_FUNCTION (this);
  sdn::MessageHeader msg;
  Time now = Simulator::Now ();
  msg.SetVTime (m_helloInterval);
  msg.SetTimeToLive (41993);//Just MY Birthday.
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  msg.SetMessageType (sdn::MessageHeader::HELLO_MESSAGE);
  msg.SetOriginatorAddress(m_CCHmainAddress);
 // std::cout<<"SendHello " <<m_mobility->GetPosition ().x<<std::endl;
  sdn::MessageHeader::Hello &hello = msg.GetHello ();
  hello.ID = m_SCHmainAddress;
  Vector pos = m_mobility->GetPosition ();
  Vector vel = m_mobility->GetVelocity ();
  hello.SetPosition (pos.x, pos.y, pos.z);
  hello.SetVelocity (vel.x, vel.y, vel.z);

  NS_LOG_DEBUG ( "SDN HELLO_MESSAGE sent by node: " << hello.ID
                 << "   at " << now.GetSeconds() << "s");

  QueueMessage (msg, JITTER);
}

void
RoutingProtocol::SendRoutingMessage (enum direction dir)
{
  NS_LOG_FUNCTION (this);

  for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
       cit != m_lc_info.end (); ++cit)
    {
    if(cit->second.dir==dir){
      sdn::MessageHeader msg;
      Time now = Simulator::Now ();
      msg.SetVTime (m_helloInterval);
      msg.SetTimeToLive (41993);//Just MY Birthday.
      msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
      msg.SetMessageType (sdn::MessageHeader::ROUTING_MESSAGE);
      msg.SetOriginatorAddress(m_CCHmainAddress);
      sdn::MessageHeader::Rm &rm = msg.GetRm ();
      //rm.ID = cit->first;//0..0
      //std::cout<<"66666 "<<m_SCHaddr2CCHaddr.size()<<std::endl;
      /*for (std::map<Ipv4Address, Ipv4Address>::const_iterator ttt = m_SCHaddr2CCHaddr.begin ();
           ttt != m_SCHaddr2CCHaddr.end (); ++ttt)
      {
          std::cout<<"6666 "<<ttt->first.Get()<<" "<<ttt->second.Get()<<std::endl;
      }*/
      std::map<Ipv4Address, Ipv4Address>::iterator ttt = m_SCHaddr2CCHaddr.find(cit->first);
      if (ttt != m_SCHaddr2CCHaddr.end ())
      {
          rm.ID = ttt->second;
          //std::cout<<"666666 "<<rm.ID.Get()<<" "<<cit->first.Get()<<std::endl;
      }
      //rm.ID = m_SCHaddr2CCHaddr[cit->first];
      //std::cout<<"666666 "<<rm.ID.Get()<<" "<<cit->first.Get()<<std::endl;
      sdn::MessageHeader::Rm::Routing_Tuple rt;
      for (std::vector<RoutingTableEntry>::const_iterator cit2 = cit->second.R_Table.begin ();
           cit2 != cit->second.R_Table.end (); ++cit2)
        {
          rt.destAddress = cit2->destAddr;
          rt.mask = cit2->mask;
          rt.nextHop = cit2->nextHop;
          rm.routingTables.push_back (rt);
        }
      rm.routingMessageSize = rm.routingTables.size ();
      if(rm.routingMessageSize==0) continue;
      QueueMessage (msg, JITTER);
    }
    }
}




void
RoutingProtocol::SendCRREQ (Ipv4Address const &destAddress)
{
  NS_LOG_FUNCTION (this);

 // std::cout<<"SendCRREQ "<<std::endl;
  sdn::MessageHeader msg;
  Time now = Simulator::Now ();
  msg.SetVTime (m_helloInterval);
  msg.SetTimeToLive (41993);//Just MY Birthday.
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  msg.SetMessageType (sdn::MessageHeader::CARROUTEREQUEST_MESSAGE);
  sdn::MessageHeader::CRREQ &crreq = msg.GetCRREQ ();
  crreq.sourceAddress=m_SCHmainAddress;
  crreq.destAddress=destAddress;
  crreq.tag=++m_tag;
  QueueMessage (msg, JITTER);
}

void
RoutingProtocol::SendCRREP( Ipv4Address const &sourceAddress,
		Ipv4Address const&destAddress, Ipv4Address const &transferAddress)
{
  NS_LOG_FUNCTION (this);

  //std::cout<<"SendCRREP "<<std::endl;
  sdn::MessageHeader msg;
  Time now = Simulator::Now ();
  msg.SetVTime (m_helloInterval);
  msg.SetTimeToLive (41993);//Just MY Birthday.
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  msg.SetMessageType (sdn::MessageHeader::CARROUTERESPONCE_MESSAGE);
  sdn::MessageHeader::CRREP &crrep = msg.GetCRREP ();
  crrep.sourceAddress=sourceAddress;
  crrep.destAddress=destAddress;
  crrep.transferAddress=transferAddress;
  QueueMessage (msg, JITTER);
}

void
RoutingProtocol::AodvTimerExpire()
{
	Aodv_sendback();
	//isDes=false;
}

void
RoutingProtocol::ProcessAodvRm(const MessageHeader &msg)
{
         //std::cout<<"ProcessAodvRm..."<<m_CCHmainAddress<<std::endl;
	 sdn::MessageHeader mesg;

	 const sdn::MessageHeader::AodvRm &aodvrm = msg.GetAodvRm();

uint16_t TTL=msg.GetTimeToLive();
if(--TTL==0) return;
//判断车流方向是否指向本路
sdn::direction in =aodvrm.dir;
sdn::direction out;
double x,y,z;
double x_0;
double y_0;


if(m_roadtype==sdn::ROW) {
    aodvrm.GetPosition(x,y,z);
    x_0=m_mobility->GetPosition().x;
    y_0=m_mobility->GetPosition().y;
}
else{
   aodvrm.GetPosition(y,x,z);
   x_0=m_mobility->GetPosition().y;
   y_0=m_mobility->GetPosition().x;
}

   if(x<x_0&&y>y_0&&in==sdn::NEGATIVE) out=sdn::POSITIVE;
   else if(x>x_0&&y>y_0&&in==sdn::NEGATIVE) out=sdn::NEGATIVE;
   else if(x>x_0&&abs(y-y_0)<0.00001&&in==sdn::NEGATIVE) out=sdn::NEGATIVE;
   else if(x>x_0&&y<y_0&&in==sdn::POSITIVE) out=sdn::NEGATIVE;
      else if(x<x_0&&y<y_0&&in==sdn::POSITIVE) out=sdn::POSITIVE;
    else if(x<x_0&&abs(y-y_0)<0.00001&&in==sdn::POSITIVE) out=sdn::POSITIVE;
      else return;
   if(out==sdn::POSITIVE&& !possive_valid) {
   //std::cout<<"no valid"<<std::endl;
   return;}
    if(out==sdn::NEGATIVE&& !negative_valid){
    //std::cout<<"no valid"<<std::endl;
    return;}
      //std::cout<<"dir: "<<out<<std::endl;

	if(!isDes&&m_lc_info.find(aodvrm.DesId)!=m_lc_info.end()){
	             std::cout<<"des get pt "<<std::endl;
                         if(!haveSink){
                            haveSink=true;
                            m_sinkAddress=aodvrm.DesId;
                            std::map<Ipv4Address, CarInfo>::iterator it1 = m_lc_positive_info.find (m_sinkAddress);
                            std::map<Ipv4Address, CarInfo>::iterator it2 = m_lc_negative_info.find (m_sinkAddress);
                            if(it1!=m_lc_positive_info.end()) m_lc_positive_info.erase(it1);
                            if(it2!=m_lc_negative_info.end()) m_lc_negative_info.erase(it2);
                            ComputeRoute();//将sink加入路由中
                            if(m_lc_info[m_sinkAddress].dir==sdn::POSITIVE){
                                if(!possive_valid) haveSink=false;
                            }
                            else {if(!negative_valid) haveSink=false;}
                         }
	
	                   if(m_incomeDesParm.desdir==sdn::OTHER)
                               {
                                 m_incomeDesParm.desdir=m_lc_info[aodvrm.DesId].dir;

                                }
                          if(m_incomeDesParm.desdir==out) {//车流方向与目的车一致
		                  temp_desId=aodvrm.DesId;
			            std::cout<<"I am  "<<m_CCHmainAddress<<" ,I have des"<<std::endl;
			            if(haveSink){
				 isDes=true;
				 //m_aodvTimer.SetDelay(FemtoSeconds (5));// 5s countdown
				 m_aodvTimer.SetFunction
				    (&RoutingProtocol::AodvTimerExpire, this);
				 Time t = Seconds (0.25);
				 m_aodvTimer.SetDelay(t);
				 m_aodvTimer.Schedule ();
				 }
				 else{
				    std::cout<<"sink no valid"<<std::endl;
				    }
				
				 }
			 }
     // std::cout<<"aodvrm.tag "<<aodvrm.tag<<" m_tag "<<m_tag<<std::endl;
        if(aodvrm.tag>m_tag){
                 m_tag=aodvrm.tag;
                 m_incomeParm_possitive.jumpnums=1000;
                 m_incomeParm_possitive.stability=1000;
                 m_incomeParm_negative.jumpnums=1000;
                 m_incomeParm_negative.stability=1000;
          }

if(out==sdn::POSITIVE)
{

	 
	 if(aodvrm.jump_nums<m_incomeParm_possitive.jumpnums||(aodvrm.jump_nums==m_incomeParm_possitive.jumpnums&& aodvrm.GetStability() < m_incomeParm_possitive.stability))
	 {//forward this packet

		 m_incomeParm_possitive.jumpnums=aodvrm.jump_nums;
		 m_incomeParm_possitive.stability=aodvrm.GetStability();
		 m_incomeParm_possitive.m_sourceId=aodvrm.ID;
		 m_incomeParm_possitive.m_desId=aodvrm.DesId;
                   m_incomeParm_possitive.lastdir=aodvrm.dir;//记录上一跳的方向	
		m_incomeParm_possitive.lastIP=aodvrm.Originator;//记录上一跳地址

		 if(!isDes){
		 //std::cout<<"forwarding..."<<std::endl;
		 mesg.SetMessageType(sdn::MessageHeader::AODV_ROUTING_MESSAGE);
		  Time now = Simulator::Now ();
		  mesg.SetVTime (m_helloInterval);
		  mesg.SetTimeToLive (TTL);
		  mesg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
		  sdn::MessageHeader::AodvRm &Aodvrm = mesg.GetAodvRm();
		  Aodvrm.tag=m_tag;
		  Aodvrm.ID=aodvrm.ID;
		  Aodvrm.DesId=aodvrm.DesId;
		  Aodvrm.mask=aodvrm.mask;
		  Aodvrm.jump_nums=m_incomeParm_possitive.jumpnums+m_selfParm_possitive.jumpnums;
		  Aodvrm.SetStability(m_incomeParm_possitive.stability>m_selfParm_possitive.stability?m_incomeParm_possitive.stability:m_selfParm_possitive.stability);
		  Aodvrm.Originator=m_CCHmainAddress;
		  Aodvrm.SetPosition(m_mobility->GetPosition().x, m_mobility->GetPosition().y,m_mobility->GetPosition().z);
		   Aodvrm.dir=out;
		   //std::cout<<"POSITIVE forwarding..."<<std::endl;
		  QueueMessage (mesg, JITTER);
		 }
	 }
	 else{
	       // std::cout<<"m_incomeParm_possitive.jumpnums "<<m_incomeParm_possitive.jumpnums<<"  m_incomeParm_possitive.stability "<< m_incomeParm_possitive.stability<<" aodvrm.jump_nums "<<aodvrm.jump_nums<<" aodvrm.GetStability()"<<aodvrm.GetStability()<<std::endl;
	 }
}
else{

	 if(aodvrm.jump_nums<m_incomeParm_negative.jumpnums||(aodvrm.jump_nums==m_incomeParm_negative.jumpnums&& aodvrm.GetStability() < m_incomeParm_negative.stability)){//forward this packet

		 m_incomeParm_negative.jumpnums=aodvrm.jump_nums;
		 m_incomeParm_negative.stability=aodvrm.GetStability();
		 m_incomeParm_negative.m_sourceId=aodvrm.ID;
		 m_incomeParm_negative.m_desId=aodvrm.DesId;
                   m_incomeParm_negative.lastdir=aodvrm.dir;//记录上一跳的方向

		 //记录上一跳地址
		m_incomeParm_negative.lastIP=aodvrm.Originator;

		 if(!isDes){
		 //std::cout<<"forwarding..."<<std::endl;
		 mesg.SetMessageType(sdn::MessageHeader::AODV_ROUTING_MESSAGE);
		  Time now = Simulator::Now ();
		  mesg.SetVTime (m_helloInterval);
		  mesg.SetTimeToLive (TTL);
		  mesg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
		  sdn::MessageHeader::AodvRm &Aodvrm = mesg.GetAodvRm();
		  Aodvrm.tag=m_tag;
		  Aodvrm.ID=aodvrm.ID;
		  Aodvrm.DesId=aodvrm.DesId;
		  Aodvrm.mask=aodvrm.mask;
		  Aodvrm.jump_nums=m_incomeParm_negative.jumpnums+m_selfParm_negative.jumpnums;
		  Aodvrm.SetStability(m_incomeParm_negative.stability>m_selfParm_negative.stability?m_incomeParm_negative.stability:m_selfParm_negative.stability);
		  Aodvrm.Originator=m_CCHmainAddress;
		  Aodvrm.SetPosition(m_mobility->GetPosition().x, m_mobility->GetPosition().y,m_mobility->GetPosition().z);
		   Aodvrm.dir=out;
		   //std::cout<<"NEGATIVE forwarding..."<<std::endl;
		  QueueMessage (mesg, JITTER);
		 }
	 }
	 	 else{
	       // std::cout<<"m_incomeParm_negative.jumpnums "<<m_incomeParm_negative.jumpnums<<"  m_incomeParm_negative.stability "<< m_incomeParm_negative.stability<<" aodvrm.jump_nums "<<aodvrm.jump_nums<<" aodvrm.GetStability()"<<aodvrm.GetStability()<<std::endl;
	 }
	 
	 }

	if(isDes){//用另外的数据结构存
   


        if(aodvrm.jump_nums<m_incomeDesParm.jumpnums||(aodvrm.jump_nums==m_incomeDesParm.jumpnums&&aodvrm.GetStability()<m_incomeDesParm.stability))
        {
           m_incomeDesParm.dir=true;
   
            m_incomeDesParm.jumpnums=aodvrm.jump_nums;
            m_incomeDesParm.stability=aodvrm.GetStability();
            m_incomeDesParm.m_sourceId=aodvrm.ID;
	   m_incomeDesParm.m_desId=aodvrm.DesId;
            m_incomeDesParm.lastdir=aodvrm.dir;//记录上一跳的方向
             m_incomeDesParm.lastIP=aodvrm.Originator;
        }
        
      }
	
}



void RoutingProtocol::ProcessAodvRERm(const sdn::MessageHeader &msg) //for each lc received Reverse message
{
   //std::cout<<"i am "<<m_CCHmainAddress<<std::endl;
	//std::map<Ipv4Address,Ipv4Address,> lc_Rtable;
    const sdn::MessageHeader::Aodv_R_Rm &Aodv_r = msg.GetAodv_R_Rm();
	if(Aodv_r.next==m_CCHmainAddress){
		std::cout<<"ProcessAodvRERm  i am "<<m_CCHmainAddress;


		

	sdn::MessageHeader mesg;
	 mesg.SetMessageType(sdn::MessageHeader::AODV_REVERSE_MESSAGE);
	  Time now = Simulator::Now ();
	  mesg.SetVTime (m_helloInterval);
	  mesg.SetTimeToLive (1234);
	  mesg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
	  sdn::MessageHeader::Aodv_R_Rm &Aodv_r_rm = mesg.GetAodv_R_Rm();

    if(Aodv_r.next_dir==sdn::POSITIVE){	  
	m_incomeParm_possitive.nextIP=Aodv_r.originator;
          if(m_lc_info.find(Aodv_r.ID)==m_lc_info.end())
         {
	        Aodv_r_rm.ID=m_incomeParm_possitive.m_sourceId; //多个流时需判断
	        Aodv_r_rm.DesId=m_incomeParm_possitive.m_desId;
	        Aodv_r_rm.FirstCarId=transferAddress_possitive;
	        Aodv_r_rm.mask=m_ipv4->GetAddress(0, 0).GetMask();
	        Aodv_r_rm.routingMessageSize=28;
	       Aodv_r_rm.originator=m_CCHmainAddress;
	       Aodv_r_rm.next=m_incomeParm_possitive.lastIP;
	       Aodv_r_rm.next_dir=m_incomeParm_possitive.lastdir;
              std::cout<<" send to "<<Aodv_r_rm.next<<std::endl;
	      QueueMessage (mesg, JITTER);
	      //ProcessCRREP(Aodv_r.FirstCarId,sdn::POSITIVE);
         }
         else std::cout<<"finish"<<std::endl;
	   ProcessCRREP(Aodv_r.FirstCarId,sdn::POSITIVE);//todo

	}
   else{
         	m_incomeParm_negative.nextIP=Aodv_r.originator;
          if(m_lc_info.find(Aodv_r.ID)==m_lc_info.end())
         {
	        Aodv_r_rm.ID=m_incomeParm_negative.m_sourceId; //多个流时需判断
	        Aodv_r_rm.DesId=m_incomeParm_negative.m_desId;
	        Aodv_r_rm.FirstCarId=transferAddress_negative;
	        Aodv_r_rm.mask=m_ipv4->GetAddress(0, 0).GetMask();
	        Aodv_r_rm.routingMessageSize=28;
	       Aodv_r_rm.originator=m_CCHmainAddress;
	       Aodv_r_rm.next=m_incomeParm_negative.lastIP;
	       Aodv_r_rm.next_dir=m_incomeParm_negative.lastdir;
              std::cout<<" send to "<<Aodv_r_rm.next<<std::endl;
	      QueueMessage (mesg, JITTER);
	      
         }
         else std::cout<<"finish"<<std::endl;
	    ProcessCRREP(Aodv_r.FirstCarId,sdn::NEGATIVE);//todo
    
            }
     }

}


void RoutingProtocol::Aodv_sendback()  //for des lc send back
{
         std::cout<<"send back"<<std::endl;
	sdn::MessageHeader msg;
	 msg.SetMessageType(sdn::MessageHeader::AODV_REVERSE_MESSAGE);
	  Time now = Simulator::Now ();
	  msg.SetVTime (m_helloInterval);
	  msg.SetTimeToLive (1234);
	  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
	  sdn::MessageHeader::Aodv_R_Rm &Aodv_r_rm = msg.GetAodv_R_Rm();
	  
	  Aodv_r_rm.ID=m_incomeDesParm.m_sourceId; 
	  Aodv_r_rm.DesId=m_incomeDesParm.m_desId;

	        if(m_incomeDesParm.desdir==sdn::POSITIVE)
	            Aodv_r_rm.FirstCarId=transferAddress_possitive;
	        else    Aodv_r_rm.FirstCarId=transferAddress_negative;

	/*  
	  if(m_incomeDesParm.dir){ //POSITIVE,NEGATIVE
	        if(m_incomeDesParm.desdir==sdn::POSITIVE)
	            Aodv_r_rm.FirstCarId=transferAddress_possitive;
	        else    Aodv_r_rm.FirstCarId=transferAddress_negative;
	            }
	   else
	   {
	    	 if(m_incomeDesParm.desdir==sdn::POSITIVE)
	            Aodv_r_rm.FirstCarId=roadendAddress_possitive;
	        else    Aodv_r_rm.FirstCarId=roadendAddress_negative;
	   }*/
	  Aodv_r_rm.mask=m_ipv4->GetAddress(0, 0).GetMask();
	  Aodv_r_rm.routingMessageSize=32;//SDN_AODVRRM_HEADER_SIZE;
	  Aodv_r_rm.originator=m_CCHmainAddress;
	  Aodv_r_rm.next=m_incomeDesParm.lastIP;
	  Aodv_r_rm.next_dir=m_incomeDesParm.lastdir;

	  QueueMessage (msg, JITTER);
          
          SendRoutingMessage(m_incomeDesParm.desdir);
	
}







void
RoutingProtocol::SetMobility (Ptr<MobilityModel> mobility)
{
  m_mobility = mobility;
}

void
RoutingProtocol::SetType (NodeType nt)
{
  m_nodetype = nt;
}

void
RoutingProtocol::SetRoadType (RoadType nt)
{
  m_roadtype = nt;
}

NodeType
RoutingProtocol::GetType () const
{
  return m_nodetype;
}

typedef struct Edge
{
    int u, v;    // 猫碌路莽聜鹿茂录聦茅聡聧莽聜鹿
    int weight;  // 猫戮鹿莽職聞忙聺聝氓聙录
} Edge;

void
RoutingProtocol::ComputeRoute ()
{

    RemoveTimeOut (); //Remove Stale Tuple

    //std::cout<<"ip "<<m_CCHmainAddress<<std::endl;
   compute_possive();
   compute_negative();
    
}//RoutingProtocol::ComputeRoute

void  RoutingProtocol::compute_possive()
{
   //std::cout<<"m_signal_range "<<m_signal_range<<" "<<"m_road_length "<<m_road_length<<std::endl;
    possive_valid=true;
    std::map<double,Ipv4Address> dis;
    std::vector<std::pair<double,Ipv4Address>> chose;

    for(std::map<Ipv4Address,CarInfo>::iterator it=m_lc_positive_info.begin();it!=m_lc_positive_info.end();++it)
    {
       if(m_roadtype==sdn::ROW)
           dis[it->second.Position.x-(m_mobility->GetPosition().x-m_road_length/2)]=it->first;
       else dis[it->second.Position.y-(m_mobility->GetPosition().y-m_road_length/2)]=it->first;
    }
    if(dis.size()==0) {
                //std::cout<<"no valid possive connect"<<std::endl;
            possive_valid=false;
            return;
    }
    //if(m_CCHmainAddress==Ipv4Address("192.168.1.247")){
       // std::cout<<"possitive dis size "<<dis.size()<<std::endl;
        // for(std::map<double,Ipv4Address>::iterator it=dis.begin();it!=dis.end();++it)
             //   std::cout<<it->first<<std::endl;
        //}
    if(dis.begin()->first>m_signal_range/2) {
    return;
    possive_valid=false;
    }
    transferAddress_possitive=dis.begin()->second;
    chose.push_back(*dis.begin());
    std::pair<double,Ipv4Address> temp=*chose.rbegin();
    
    while(temp.first+m_signal_range/2<m_road_length)
    {
        
        uint32_t  t=chose.size();
       //std::cout<<"2"<<std::endl;
        std::map<double,Ipv4Address>::iterator iter=dis.find(temp.first);

        
        while(++iter!=dis.end())
        {

        //std::cout<<"4"<<std::endl;
            std::pair<double,Ipv4Address> target=*(iter);
            if(temp.first+m_signal_range>iter->first)
            {
                std::map<double,Ipv4Address>::iterator it_temp=iter;
                if(++it_temp==dis.end())
                {
                    chose.push_back(target);
                               // if(m_CCHmainAddress==Ipv4Address("192.168.1.247")){
          // std::cout<<"push "<<target.first<<std::endl;
       // }
                    break;
                }
                else if(temp.first+m_signal_range<(it_temp)->first)
                {
                    chose.push_back(target);
                                                    //if(m_CCHmainAddress==Ipv4Address("192.168.1.247")){
           //std::cout<<"push "<<target.first<<std::endl;
       // }
                    break;
                }
                
            }

        }//while(++iter!
        //std::cout<<"5"<<std::endl;
        if(t==chose.size())
        {
            //std::cout<<"no valid possive connect"<<std::endl;
            possive_valid=false;
            return;
            //break;
        }
        temp=*chose.rbegin();
    }//while(temp.first+m...
           // if(m_CCHmainAddress==Ipv4Address("192.168.1.247")){
           //std::cout<<"valid"<<std::endl;
       // }

/*for(std::vector<std::pair<double,Ipv4Address>>::iterator it=chose.begin();it!=chose.end();it++)
        std::cout<<it->first<<"->";


        std::cout<<std::endl;*/
    
    //std::cout<<"1"<<std::endl;
    roadendAddress_possitive=chose.rbegin()->second;
    Ipv4Address mask("255.255.0.0");
    double mean=0;
    double sd=0;
    for(std::vector<std::pair<double,Ipv4Address>>::iterator it = chose.begin();it!=chose.end();++it)
    {
        LCAddEntry (it->second, chose.rbegin()->second, mask, (it+1)->second);//更新每个carinfo的R_Table
        mean+=it->first;
    }
    m_selfParm_possitive.jumpnums=chose.size();
    //stablity to do
    mean=mean/chose.size();//求速度的平均值
        for(std::vector<std::pair<double,Ipv4Address>>::iterator it = chose.begin();it!=chose.end();++it)
    {
        sd+=(it->first-mean)*(it->first-mean);
    }
    sd=sd/chose.size();
    sd=sqrt(sd);//求速度的标准差
    m_selfParm_possitive.stability=sd/mean;


    if(haveSource)
    {
        if(m_lc_info[m_sourceAddress].dir==sdn::POSITIVE)
        {
            if(m_roadtype==sdn::ROW)
            {
                int distance=m_lc_info[m_sourceAddress].Position.x-(m_mobility->GetPosition().x-m_road_length/2);
                for(std::vector<std::pair<double,Ipv4Address>>::iterator it = chose.begin();it!=chose.end();++it)
                {
                    if(it->first>distance){
                        LCAddEntry (m_sourceAddress,chose.rbegin()->second, mask, (it)->second);
                        break;
                    }
                    
                }
                
            }
            else{
                int distance=m_lc_info[m_sourceAddress].Position.y-(m_mobility->GetPosition().y-m_road_length/2);
                 for(std::vector<std::pair<double,Ipv4Address>>::iterator it = chose.begin();it!=chose.end();++it)
                {
                    if(it->first>distance){
                        LCAddEntry (m_sourceAddress,chose.rbegin()->second, mask, (it)->second);
                        break;
                    }
                    
                }
            }
        }
    }
    if(haveSink)
    {
        if(m_lc_info[m_sinkAddress].dir==sdn::POSITIVE)
        {
            if(m_roadtype==sdn::ROW)
            {
                int distance=m_lc_info[m_sinkAddress].Position.x-(m_mobility->GetPosition().x-m_road_length/2);
               Ipv4Address temp;
                 for(std::vector<std::pair<double,Ipv4Address>>::reverse_iterator it = chose.rbegin();it!=chose.rend();++it)
                {
                    if(it->first<distance){
                    //std::cout<<"sink add"<<std::endl;
                       temp=it->second;
                        //LCAddEntry (it->second,chose.rbegin()->second, mask, m_sinkAddress);
                        break;
                    }
                    
                }
                 for(std::vector<std::pair<double,Ipv4Address>>::iterator it = chose.begin();it!=chose.end();++it)
                {
                    if(it->first<distance){
                        //std::cout<<"sink add"<<std::endl;
                        if(it->second==temp) LCAddEntry (it->second,m_sinkAddress,mask,m_sinkAddress);
                        else LCAddEntry (it->second,m_sinkAddress,mask,(it+1)->second);
 
                    }
                    else break;
                    
                }
                
            }
            else{
                int distance=m_lc_info[m_sinkAddress].Position.y-(m_mobility->GetPosition().y-m_road_length/2);
              

               Ipv4Address temp;
                 for(std::vector<std::pair<double,Ipv4Address>>::reverse_iterator it = chose.rbegin();it!=chose.rend();++it)
                {
                    if(it->first<distance){
                    //std::cout<<"sink add"<<std::endl;
                       temp=it->second;
                        //LCAddEntry (it->second,chose.rbegin()->second, mask, m_sinkAddress);
                        break;
                    }
                    
                }
                 for(std::vector<std::pair<double,Ipv4Address>>::iterator it = chose.begin();it!=chose.end();++it)
                {
                    if(it->first<distance){
                        //std::cout<<"sink add"<<std::endl;
                        if(it->second==temp) LCAddEntry (it->second,m_sinkAddress,mask,m_sinkAddress);
                        else LCAddEntry (it->second,m_sinkAddress,mask,(it+1)->second);
 
                    }
                    else break;
                    
                }
            }            
        }        
    }
    
}

void RoutingProtocol::compute_negative()
{
     //std::cout<<"compute negative"<<std::endl;
       negative_valid=true;
        std::map<double,Ipv4Address> dis;
 std::vector<std::pair<double,Ipv4Address>> chose;
    for(std::map<Ipv4Address,CarInfo>::iterator it=m_lc_negative_info.begin();it!=m_lc_negative_info.end();++it)
    {
       if(m_roadtype==sdn::ROW)
           dis[m_mobility->GetPosition().x+m_road_length/2-it->second.Position.x]=it->first;
       else dis[m_mobility->GetPosition().y+m_road_length/2-it->second.Position.y]=it->first;
    }
    if(dis.size()==0) {
            //std::cout<<"no valid negative connect"<<std::endl;
            negative_valid=false;
            return;
    } 
    if(dis.begin()->first>m_signal_range/2) {
    return;
    possive_valid=false;
    }
        /*std::cout<<"negative dis size "<<dis.size()<<std::endl;
    for(std::map<double,Ipv4Address>::iterator it=dis.begin();it!=dis.end();++it)
        std::cout<<it->first<<std::endl;*/
    transferAddress_negative=dis.begin()->second;
    chose.push_back(*dis.begin());
    std::pair<double,Ipv4Address> temp=*chose.rbegin();
    
    while(temp.first+m_signal_range/2<m_road_length)
    {
    uint32_t t=chose.size();
       //std::cout<<"2"<<std::endl;
        std::map<double,Ipv4Address>::iterator iter=dis.find(temp.first);

        
        while(++iter!=dis.end())
        {
        //std::cout<<"4"<<std::endl;
            std::pair<double,Ipv4Address> target=*(iter);
            if(temp.first+m_signal_range>iter->first)
            {
                std::map<double,Ipv4Address>::iterator it_temp=iter;
                if(++it_temp==dis.end())
                {
                    chose.push_back(target);
                    break;
                }
                else if(temp.first+m_signal_range<(it_temp)->first)
                {
                    chose.push_back(target);
                    break;
                }
                
            }

        }//while(++iter!
        //std::cout<<"5"<<std::endl;
        if(t==chose.size())
        {
            //std::cout<<"no valid negative connect"<<std::endl;
            possive_valid=false;
            return;
            //break;
        }
        temp=*chose.rbegin();
    }
    
    roadendAddress_negative=chose.rbegin()->second;
        Ipv4Address mask("255.255.0.0");
     double mean=0;
    double sd=0;
    for(std::vector<std::pair<double,Ipv4Address>>::iterator it = chose.begin();it!=chose.end();++it)
    {
        LCAddEntry (it->second, chose.rbegin()->second, mask, (it+1)->second);
                mean+=it->first;
    }
        m_selfParm_negative.jumpnums=chose.size();
 
    mean=mean/chose.size();//求速度的平均值
        for(std::vector<std::pair<double,Ipv4Address>>::iterator it = chose.begin();it!=chose.end();++it)
    {
        sd+=(it->first-mean)*(it->first-mean);
    }
    sd=sd/chose.size();
    sd=sqrt(sd);//求速度的标准差
    m_selfParm_negative.stability=sd/(-1*mean);



    if(haveSource)
    {
        if(m_lc_info[m_sourceAddress].dir==sdn::NEGATIVE)
        {
            if(m_roadtype==sdn::ROW)
            {
                int distance=m_mobility->GetPosition().x+m_road_length/2-m_lc_info[m_sourceAddress].Position.x;
                for(std::vector<std::pair<double,Ipv4Address>>::iterator it = chose.begin();it!=chose.end();++it)
                {
                    if(it->first>distance){
                        LCAddEntry (m_sourceAddress,chose.rbegin()->second, mask, (it)->second);
                        break;
                    }
                    
                }
                
            }
            else{
                int distance=m_mobility->GetPosition().y+m_road_length/2-m_lc_info[m_sourceAddress].Position.y;
                 for(std::vector<std::pair<double,Ipv4Address>>::iterator it = chose.begin();it!=chose.end();++it)
                {
                    if(it->first>distance){
                        LCAddEntry (m_sourceAddress,chose.rbegin()->second, mask, (it)->second);
                        break;
                    }
                    
                }
            }
        }
    }
    if(haveSink)
    {
        if(m_lc_info[m_sinkAddress].dir==sdn::NEGATIVE)
        {
            if(m_roadtype==sdn::ROW)
            {
                int distance=m_mobility->GetPosition().x+m_road_length/2-m_lc_info[m_sinkAddress].Position.x;
               Ipv4Address temp;
                 for(std::vector<std::pair<double,Ipv4Address>>::reverse_iterator it = chose.rbegin();it!=chose.rend();++it)
                {
                    if(it->first<distance){
                    //std::cout<<"sink add"<<std::endl;
                       temp=it->second;
                        //LCAddEntry (it->second,chose.rbegin()->second, mask, m_sinkAddress);
                        break;
                    }
                    
                }
                 for(std::vector<std::pair<double,Ipv4Address>>::iterator it = chose.begin();it!=chose.end();++it)
                {
                    if(it->first<distance){
                        //std::cout<<"sink add"<<std::endl;
                        if(it->second==temp) LCAddEntry (it->second,m_sinkAddress,mask,m_sinkAddress);
                        else LCAddEntry (it->second,m_sinkAddress,mask,(it+1)->second);
 
                    }
                    else break;
                    
                }
                
            }
            else{
                int distance=m_mobility->GetPosition().y+m_road_length/2-m_lc_info[m_sinkAddress].Position.y;
               Ipv4Address temp;
                 for(std::vector<std::pair<double,Ipv4Address>>::reverse_iterator it = chose.rbegin();it!=chose.rend();++it)
                {
                    if(it->first<distance){
                    //std::cout<<"sink add"<<std::endl;
                       temp=it->second;
                        //LCAddEntry (it->second,chose.rbegin()->second, mask, m_sinkAddress);
                        break;
                    }
                    
                }
                 for(std::vector<std::pair<double,Ipv4Address>>::iterator it = chose.begin();it!=chose.end();++it)
                {
                    if(it->first<distance){
                        //std::cout<<"sink add"<<std::endl;
                        if(it->second==temp) LCAddEntry (it->second,m_sinkAddress,mask,m_sinkAddress);
                        else LCAddEntry (it->second,m_sinkAddress,mask,(it+1)->second);
 
                    }
                    else break;
                    
                }
            }            
        }        
    }

    
}

/*
void
RoutingProtocol::Do_Init_Compute ()
{
  //std::cout<<"Partition"<<std::endl;
  Partition ();
  //std::cout<<"SetN_Init"<<std::endl;
  SetN_Init ();
  //std::cout<<"OtherSet_Init"<<std::endl;
  OtherSet_Init ();
  //std::cout<<"SelectNode"<<std::endl;
  SelectNode ();
  //std::cout<<"Do_Init_Compute DONE"<<std::endl;
}

void
RoutingProtocol::Do_Update ()
{
  //std::cout<<"ShiftArea"<<std::endl;
  ShiftArea ();
  //std::cout<<"AddNewToZero"<<std::endl;
  AddNewToZero ();
  //std::cout<<"CalcSetZero"<<std::endl;
  CalcSetZero ();
  //std::cout<<"SelectNewNodeInAreaZero"<<std::endl;
  SelectNewNodeInAreaZero ();
  //std::cout<<"Do_Update DONE"<<std::endl;
}

void
RoutingProtocol::Partition ()
{
  m_Sections.clear ();
  int numArea = GetNumArea();
  for (int i = 0; i < numArea; ++i)
    {
      m_Sections.push_back (std::set<Ipv4Address> ());
    }
  //std::cout<<"CheckPonint1"<<std::endl;
  for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();cit != m_lc_info.end(); ++cit)
    {
      //std::cout<<"cit->first"<<cit->first.Get ()%256<<std::endl;
      //std::cout<<GetArea (cit->second.Position)<<","<<numArea<<std::endl;
      m_Sections[GetArea (cit->second.Position)].insert (cit->first);
    }
  //std::cout<<m_lc_info.size ()<<std::endl;
  for (int i = 0; i < numArea; ++i)
    {
      //std::cout<<"Section "<<i<<": ";
      for (std::set<Ipv4Address>::const_iterator cit = m_Sections[i].begin ();
           cit != m_Sections[i].end (); ++cit)
        {
          //std::cout<<cit->Get ()%256<<",";
        }
      //std::cout<<std::endl;
    }

}

void
RoutingProtocol::SetN_Init ()
{
  int numArea = GetNumArea();
  for (std::set<Ipv4Address>::const_iterator cit = m_Sections[numArea-1].begin ();
      cit != m_Sections[numArea-1].end (); ++cit)
    {
      m_lc_info[(*cit)].minhop = 1;
      m_lc_info[(*cit)].ID_of_minhop = Ipv4Address::GetZero ();
    }
}

void
RoutingProtocol::OtherSet_Init ()
{
  int numArea = GetNumArea();
  m_lc_info.clear ();
  for (int area = numArea - 2; area >= 0; --area)
    {
      m_lc_shorthop.clear();
      SortByDistance (area);
      CalcShortHopOfArea (area, area + 1);
      if ((area == numArea - 3) && isPaddingExist ())
        {
          CalcShortHopOfArea (area, area + 2);
        }
      CalcIntraArea (area);
    }
}

void
RoutingProtocol::SortByDistance (int area)
{
  m_list4sort.clear ();
  for (std::set<Ipv4Address>::const_iterator cit = m_Sections[area].begin ();
      cit != m_Sections[area].end (); ++cit)
    {
      bool done = false;
      for (std::list<Ipv4Address>::iterator it = m_list4sort.begin ();
           it != m_list4sort.end (); ++it)
        {
          if (m_lc_info[*it].GetPos ().x < m_lc_info[*cit].GetPos ().x)
            {
              m_list4sort.insert (it, *cit);
              done = true;
              break;
            }
        }
      if (!done)
        {
          m_list4sort.push_back (*cit);
        }
    }
}

void
RoutingProtocol::CalcShortHopOfArea (int fromArea, int toArea)
{
  for (std::list<Ipv4Address>::const_iterator cit = m_list4sort.begin ();
       cit != m_list4sort.end (); ++cit)
    {
      for (std::set<Ipv4Address>::const_iterator cit2 = m_Sections[toArea].begin ();
           cit2 != m_Sections[toArea].end (); ++cit2)
        {
          m_lc_shorthop[*cit].push_back (GetShortHop (*cit,*cit2));
        }

      UpdateMinHop (*cit);
    }
}

void
RoutingProtocol::UpdateMinHop (const Ipv4Address &ID)
{
  uint32_t theminhop = INFHOP;
  Ipv4Address IDofminhop;
  for (std::list<ShortHop>::const_iterator cit = m_lc_shorthop[ID].begin ();
       cit != m_lc_shorthop[ID].end (); ++cit)
    {
      if (cit->hopnumber < theminhop)
        {
          theminhop = cit->hopnumber;
          if (cit->isTransfer)
            {
              IDofminhop = cit->proxyID;
            }
          else
            {
              IDofminhop = cit->nextID;
            }
        }
    }
  m_lc_info[ID].ID_of_minhop = IDofminhop;
  m_lc_info[ID].minhop = theminhop;
}

void
RoutingProtocol::CalcIntraArea (int area)
{
  CalcShortHopOfArea (area, area);
}

void
RoutingProtocol::SelectNode ()
{
  //4-1
  ResetAppointmentResult ();
  uint32_t thezero = 0;
  Ipv4Address The_Car(thezero);
  uint32_t minhop_of_tc = INFHOP;

  //First Area
  for (std::set<Ipv4Address>::const_iterator cit = m_Sections[0].begin ();
      cit != m_Sections[0].end (); ++cit)
    {
      CarInfo& temp_info = m_lc_info[*cit];
      if (temp_info.minhop < minhop_of_tc)
        {
          minhop_of_tc = temp_info.minhop;
          The_Car = *cit;
        }
    }
  m_theFirstCar = The_Car;
  Ipv4Address ZERO = Ipv4Address::GetZero ();
  //std::cout<<"Chain ";
  while (The_Car != ZERO)
    {
      std::cout<<The_Car.Get () % 256<<",";
      m_lc_info[The_Car].appointmentResult = FORWARDER;
      The_Car = m_lc_info[The_Car].ID_of_minhop;
    }
  std::cout<<std::endl;
}

void
RoutingProtocol::ResetAppointmentResult ()
{
  for (std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info.begin ();
       it != m_lc_info.end (); ++it)
    {
      it->second.appointmentResult = NORMAL;
    }
}

void
RoutingProtocol::ShiftArea ()
{
  for (int i = GetNumArea () - 1; i>0; --i)
    {
      m_Sections[i] = m_Sections[i-1];
    }
  m_Sections[0].clear ();
}

void
RoutingProtocol::AddNewToZero ()
{
  for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
       cit != m_lc_info.end (); ++cit)
    {
      if (GetArea (cit->second.Position) == 0)
        {
          m_Sections[0].insert(cit->first);
        }
    }
}

void
RoutingProtocol::CalcSetZero ()
{
  m_lc_shorthop.clear();
  if (GetNumArea () > 1)
    CalcShortHopOfArea (0,1);
  if ((GetNumArea () == 3)&&(isPaddingExist ()))
    CalcShortHopOfArea (0,2);
  CalcIntraArea (0);
}

void
RoutingProtocol::SelectNewNodeInAreaZero ()
{
  uint32_t thezero = 0;
  Ipv4Address The_Car (thezero);
  uint32_t minhop_of_tc = INFHOP;
  for (std::set<Ipv4Address>::const_iterator cit = m_Sections[0].begin ();
       cit != m_Sections[0].end (); ++cit)
    {
      CarInfo& temp_info = m_lc_info[*cit];
      if (temp_info.minhop < minhop_of_tc)
        {
          minhop_of_tc = temp_info.minhop;
          The_Car = *cit;
        }
      else
        if (temp_info.minhop == minhop_of_tc)
          {
            if (temp_info.ID_of_minhop == m_theFirstCar)
              {
                minhop_of_tc = temp_info.minhop;
                The_Car = *cit;
              }
          }
    }

  if (m_lc_info[The_Car].ID_of_minhop == m_theFirstCar)
    {
      m_theFirstCar = The_Car;
      m_lc_info[The_Car].appointmentResult = FORWARDER;
    }
  else
    {
      ResetAppointmentResult ();
      m_theFirstCar = The_Car;
      while (m_lc_info.find (The_Car) != m_lc_info.end ())
        {
          m_lc_info[The_Car].appointmentResult = FORWARDER;
          The_Car = m_lc_info[The_Car].ID_of_minhop;
        }
    }
}*/

void
RoutingProtocol::Reschedule ()
{
  if (m_theFirstCar == Ipv4Address::GetZero ())
    {
      if (m_apTimer.IsRunning ())
        {
          m_apTimer.Remove ();
        }
      m_apTimer.Schedule (m_minAPInterval);
    }
  else
    {
      double vx = m_lc_info[m_theFirstCar].Velocity.x;
      double px = m_lc_info[m_theFirstCar].GetPos ().x;
      double t2l;
      if (vx == 0)
        {
          t2l = 1;
        }
      else
        {
          t2l= (0.5 * m_signal_range - px) / vx;
        }
      if (m_apTimer.IsRunning ())
        {
          m_apTimer.Remove ();
        }
      m_apTimer.Schedule(Seconds(t2l));
    }
}

ShortHop
RoutingProtocol::GetShortHop(const Ipv4Address& IDa, const Ipv4Address& IDb)
{
  double const vxa = m_lc_info[IDa].Velocity.x,
               vxb = m_lc_info[IDb].Velocity.x;
  //Predict
  double const pxa = m_lc_info[IDa].GetPos ().x,
               pxb = m_lc_info[IDb].GetPos ().x;
  // time to b left
  double temp;
  if (vxb > 0)
    {
      temp = (m_road_length - pxb) / vxb;
    }
  else
    {
      //b is fixed.
      temp = (m_road_length - pxa) / vxa;
    }
  double const t2bl = temp;
  if ((pxb - pxa < m_signal_range) && (abs((pxb + vxb*t2bl)-(pxa + vxa*t2bl)) < m_signal_range))
    {
      ShortHop sh;
      sh.nextID = IDb;
      sh.hopnumber = m_lc_info[IDb].minhop + 1;
      sh.isTransfer = false;
      return sh;
    }//if ((pxb -  ...
  else
    {
      ShortHop sh;
      sh.isTransfer = true;
      sh.t = 0; // Time when connection loss
      sh.hopnumber = INFHOP;
      if (pxb - pxa < m_signal_range)
        {
          if (vxb > vxa)
            {
              sh.t = (m_signal_range + pxa - pxb) / (vxb - vxa);
            }
          else
            {
              sh.t = (m_signal_range + pxb - pxa) / (vxa - vxb);
            }
        }
      //Find another car
      for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
           cit != m_lc_info.end (); ++cit)
        {
          double const vxc = cit->second.Velocity.x;
          //pxc when t
          double const tpxc = cit->second.GetPos ().x + vxc * sh.t;
          //pxa and pxb when t
          double const tpxa = pxa + vxa * sh.t,
                       tpxb = pxb + vxb * sh.t;
          //t2bl minus t
          double const t2blmt = t2bl - sh.t;
          if ((tpxa<tpxc)&&(tpxc<tpxb))
            {
              if ((abs((tpxb + vxb*t2blmt)-(tpxc + vxc*t2blmt)) < m_signal_range)&&
                  abs((tpxc + vxc*t2blmt)-(tpxa + vxa*t2blmt)) < m_signal_range)
                {
                  sh.IDa = IDa;
                  sh.IDb = IDb;
                  sh.proxyID = cit->first;
                  sh.hopnumber = m_lc_info[IDb].minhop + 2;
                  return sh;
                }//if ((abs((tpxb ...
            }//if ((tpxa ...
        }//for (std::map<I ...
      return sh;
    }//else
}
void
RoutingProtocol::LCAddEntry (const Ipv4Address& ID,
                           const Ipv4Address &dest,
                           const Ipv4Address &mask,
                           const Ipv4Address &next,
                           uint32_t interface)
{
  NS_LOG_FUNCTION(this  << ID << dest << next << interface << mask << m_CCHmainAddress);
  //std::cout<<"dest:"<<m_next_forwarder.Get () % 256<<std::endl;
  CarInfo& Entry = m_lc_info[ID];std::cout<<"Interfaces:"<<std::endl;
  RoutingTableEntry RTE;
  RTE.destAddr = dest;
  RTE.mask = mask;
  RTE.nextHop = next;
  RTE.interface = interface;
  //remove repeat
  for(std::vector<RoutingTableEntry>::iterator it=Entry.R_Table.begin();it!=Entry.R_Table.end();++it)
  {
      if(it->destAddr == dest)
      {
              it =  Entry.R_Table.erase(it);//it point to next element;
              --it;
      }
  }  
  Entry.R_Table.push_back (RTE);
}

void
RoutingProtocol::LCAddEntry (const Ipv4Address& ID,
                           const Ipv4Address &dest,
                           const Ipv4Address &mask,
                           const Ipv4Address &next,
                           const Ipv4Address &interfaceAddress)
{
  NS_LOG_FUNCTION(this << ID << dest << next << interfaceAddress << mask << m_CCHmainAddress);

  NS_ASSERT (m_ipv4);
  std::cout<<"GetNInterfaces:"<<m_ipv4->GetNInterfaces()<<std::endl;
  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces(); ++i)
   for (uint32_t j = 0; j< m_ipv4->GetNAddresses(i); ++j)
     {
       if (m_ipv4->GetAddress(i,j).GetLocal() == interfaceAddress)
         {
           std::cout<<"GetNInterfaces:"<<i<<std::endl;
           LCAddEntry(ID, dest, mask, next, i);
           return;
         }
     }
  //ERROR NO MATCHING INTERFACES
  NS_ASSERT(false);
}
void
RoutingProtocol::LCAddEntry(const Ipv4Address& ID,
                            const Ipv4Address& dest,
                            const Ipv4Address& mask,
                            const Ipv4Address& next)
{
  CarInfo& Entry = m_lc_info[ID];
  RoutingTableEntry RTE;
  RTE.destAddr = dest;
  RTE.mask = mask;
  RTE.nextHop = next;
  RTE.interface = 0;
  //remove repeat
  for(std::vector<RoutingTableEntry>::iterator it=Entry.R_Table.begin();it!=Entry.R_Table.end();++it)
  {
      if(it->destAddr == dest)
      {
              it =  Entry.R_Table.erase(it);//it point to next element;
              --it;
      }
  }  
  Entry.R_Table.push_back (RTE);
}

void
RoutingProtocol::ClearAllTables ()
{
  for (std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info.begin (); it!=m_lc_info.end(); ++it)
    {
      for(std::vector<RoutingTableEntry>::iterator iit = it->second.R_Table.begin (); iit!=it->second.R_Table.end(); ++iit)
    {
        if(iit->destAddr != it->first)
        {
                iit = it->second.R_Table.erase(iit);//iit will point to next element;
                --iit;
        }
    }
      //it->second.R_Table.clear ();
    }
}

int
RoutingProtocol::GetArea (Vector3D position) const
{
  double &px = position.x;
  double road_length = m_road_length;
  //0.5r ~ r ~ r ~...~ r ~ r ~ last (if length_of_last<=0.5r, last={0.5r}; else last = {padding_area, 0.5r});
  if (px < 0.5*m_signal_range)
    {
      //std::cout<<"RET1"<<std::endl;
      return 0;
    }
  else
    {
      road_length -= 0.5*m_signal_range;
      //px -= 0.5*m_signal_range;
      int numOfTrivialArea = road_length / m_signal_range;
      double remain = road_length - (numOfTrivialArea * m_signal_range);
      if (!(remain>0))
        numOfTrivialArea--;

      px -= 0.5*m_signal_range;
      if (px < numOfTrivialArea * m_signal_range)
        {
          return (px / m_signal_range) + 1;
        }
      else
        {
          if (road_length - px < 0.5*m_signal_range)
            {
              if (isPaddingExist())
                return numOfTrivialArea + 2;
              else
                return numOfTrivialArea + 1;
            }
          else
            {
              return numOfTrivialArea + 1;
            }
        }
    }

}

int
RoutingProtocol::GetNumArea () const
{
  return m_numArea;
}

void
RoutingProtocol::Init_NumArea ()
{
  int ret;
  double road_length = m_road_length;
  if (road_length < 0.5*m_signal_range)
    {
      ret = 1;
    }
  else
    {
      road_length -= 0.5*m_signal_range;
      int numOfTrivialArea = road_length / m_signal_range;
      double last_length = road_length - (m_signal_range * numOfTrivialArea);
      if (last_length < 1e-10)//last_length == 0 Devied the last TrivialArea into 2
        {
          ret = 1 + (numOfTrivialArea - 1) + 1 + 1;//First Area + TrivialArea-1 + Padding + LastArea;
          m_isPadding = true;
        }
      else
        if (last_length > 0.5*m_signal_range)//0.5r<last_length<r
          {
            ret = 1 + numOfTrivialArea + 2;//First Area + TrivialArea + paddingArea +LastArea;
            m_isPadding = true;
          }
        else//0<last_length<0.5r
          {
            ret = 1 + numOfTrivialArea + 1;//First Area + TrivialArea + LastArea;
            m_isPadding = false;
          }
    }
  m_numArea = ret;
  m_numAreaVaild = true;
}

bool
RoutingProtocol::isPaddingExist () const
{
  return m_isPadding;
}

void
RoutingProtocol::RemoveTimeOut()//删除3个hello时间内没再次收到hello包的车
{
  Time now = Simulator::Now ();
  std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info.begin ();
  //std::vector<Ipv4Address> pendding;
  while (it != m_lc_info.end ())
    {
      if (now.GetSeconds() - it->second.LastActive.GetSeconds () > 3 * m_helloInterval.GetSeconds())
        {
          //pendding.push_back (it->first);
          m_lc_info.erase((it++));
        }
      else ++it;
    }
  /*for (std::vector<Ipv4Address>::iterator it = pendding.begin ();
      it != pendding.end(); ++it)
    {
      m_lc_info.erase((*it));
    }*/
    std::map<Ipv4Address, CarInfo>::iterator it_possitive = m_lc_positive_info.begin ();
      while (it_possitive != m_lc_positive_info.end ())
    {
      if (now.GetSeconds() - it_possitive->second.LastActive.GetSeconds () > 3 * m_helloInterval.GetSeconds())
        {
          m_lc_positive_info.erase((it_possitive++));
        }
      else ++it_possitive;
    }
    std::map<Ipv4Address, CarInfo>::iterator it_negative = m_lc_negative_info.begin ();
      while (it_negative != m_lc_negative_info.end ())
    {
      if (now.GetSeconds() - it_negative->second.LastActive.GetSeconds () > 3 * m_helloInterval.GetSeconds())
        {
           m_lc_negative_info.erase((it_negative++));
        }
      else ++it_negative;
    }    
}

void
RoutingProtocol::SetSignalRangeNRoadLength (double signal_range, double road_length)
{
  m_signal_range = signal_range;
  m_road_length = road_length;
}

} // namespace sdn
} // namespace ns3


