/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 goals lee
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
 * Author: goals.lee  <goals.lee@qq.com>
 */

#ifndef SDN_HEADER_H
#define SDN_HEADER_H

#include <stdint.h>
#include <vector>
#include "ns3/header.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"

namespace ns3 {
namespace sdn {


enum direction{POSITIVE,NEGATIVE,OTHER};
float     rIEEE754 (uint32_t emf);
uint32_t  IEEE754 (float dec);

// Packet Format
//
//    The basic layout of any packet in SDN is as follows (omitting IP and
//    UDP headers):
//
//        0                   1                   2                   3
//        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |                          originator                           |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |         Packet Length         |    Packet Sequence Number     |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |  Message Type |     Vtime     |         Message Size          |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |          Time To Live         |    Message Sequence Number    |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |                                                               |
//       :                            MESSAGE                            :
//       |                                                               |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |  Message Type |     Vtime     |         Message Size          |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |          Time To Live         |    Message Sequence Number    |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |                                                               |
//       :                            MESSAGE                            :
//       |                                                               |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       :                                                               :
//                (etc.)
class PacketHeader : public Header
{
public:
  PacketHeader ();
  virtual ~PacketHeader ();

  Ipv4Address originator;

  void SetPacketLength (uint16_t length)
  {
    m_packetLength = length;
  }
  uint16_t GetPacketLength () const
  {
    return (m_packetLength);
  }

  void SetPacketSequenceNumber (uint16_t seqnum)
  {
    m_packetSequenceNumber = seqnum;
  }
  uint16_t GetPacketSequenceNumber () const
  {
    return (m_packetSequenceNumber);
  }

private:
  uint16_t m_packetLength;
  uint16_t m_packetSequenceNumber;

public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
};


//        0                   1                   2                   3
//        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |                          originator                           |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |  Message Type |     Vtime     |         Message Size          |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |          Time To Live         |    Message Sequence Number    |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |                                                               |
//       :                            MESSAGE                            :
//       |                                                               |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

class MessageHeader : public Header
{
public:

  enum MessageType {
    HELLO_MESSAGE,
    ROUTING_MESSAGE,
    AODV_ROUTING_MESSAGE , //for aodv routing
    AODV_REVERSE_MESSAGE,//for aodv reverse routing
    MAINTAINMENT_MESSAGE,
    CARROUTEREQUEST_MESSAGE,
    CARROUTERESPONCE_MESSAGE
  };

  MessageHeader ();
  virtual ~MessageHeader ();

  void SetMessageType (MessageType messageType)
  {
    m_messageType = messageType;
  }
  MessageType GetMessageType () const
  {
    return (m_messageType);
  }

  void SetVTime (Time time)
  {
    //vTime is useless by far
    m_vTime = 0;
  }
  Time GetVTime () const
  {
    return Seconds(m_vTime);
  }

  void SetTimeToLive (uint16_t timeToLive)
  {
    m_timeToLive = timeToLive;
  }
  uint16_t GetTimeToLive () const
  {
    return (m_timeToLive);
  }

  void SetMessageSequenceNumber (uint16_t messageSequenceNumber)
  {
    m_messageSequenceNumber = messageSequenceNumber;
  }
  uint16_t GetMessageSequenceNumber () const
  {
    return (m_messageSequenceNumber);
  }

   void SetMessageSize (uint16_t messageSize)
   {
     m_messageSize = messageSize;
   }
   uint16_t GetMessageSize () const
   {
     return (m_messageSize);
   }
   void SetOriginatorAddress (Ipv4Address originatorAddress)
   {
      m_originatorAddress = originatorAddress;
   }
   Ipv4Address GetOriginatorAddress () const
   {
      return (m_originatorAddress);
   }
private:
  Ipv4Address m_originatorAddress;  
  MessageType m_messageType;
  uint8_t m_vTime;
  uint16_t m_timeToLive;
  uint16_t m_messageSequenceNumber;
  uint16_t m_messageSize;

public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);

  //  HELLO Message Format
  //
  //        0                   1                   2                   3
  //        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                        ID (IP Address)                        |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Position X                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Position Y                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Position Z                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Velocity X                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Velocity Y                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Velocity Z                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       :                                                               :
  //       :                                       :
  //    (etc.)
  struct Hello
  {
    Ipv4Address ID;
    
    struct Position{
      uint32_t X, Y, Z;
    };
    
    struct Velocity{
      uint32_t X, Y, Z;
    };
    
    Position position;
    void SetPosition(double x, double y, double z)
    {
      this->position.X = IEEE754(x);
      this->position.Y = IEEE754(y);
      this->position.Z = IEEE754(z);
    }
    
    void GetPosition(double &x, double &y, double &z) const
    {
      x = rIEEE754(this->position.X);
      y = rIEEE754(this->position.Y);
      z = rIEEE754(this->position.Z);
    }
    
    Vector3D GetPosition() const
    {
      return Vector3D(rIEEE754(this->position.X),
                      rIEEE754(this->position.Y),
                      rIEEE754(this->position.Z));
    }

    Velocity velocity;
    void SetVelocity(double x, double y, double z)
    {
      this->velocity.X = IEEE754(x);
      this->velocity.Y = IEEE754(y);
      this->velocity.Z = IEEE754(z);
    }
    
    void GetVelocity(double &x, double &y, double &z) const
    {
      x = rIEEE754(this->velocity.X);
      y = rIEEE754(this->velocity.Y);
      z = rIEEE754(this->velocity.Z);
    }
    
    Vector3D GetVelocity() const
    {
      return Vector3D(rIEEE754(this->velocity.X),
                      rIEEE754(this->velocity.Y),
                      rIEEE754(this->velocity.Z));
    }

    void Print (std::ostream &os) const;
    uint32_t GetSerializedSize (void) const;
    void Serialize (Buffer::Iterator start) const;
    uint32_t Deserialize (Buffer::Iterator start, uint32_t messageSize);
  };

  //  Routing Message Format
  //    One RM is for one car only.
  //    But one car may need multiple RM to form proper routing table.
  //    The proposed format of a routing message is as follows:
  //
  //        0                   1                   2                   3
  //        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                     Routing Message Size                      |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                      sourceAddress(ID)                        |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                          destAddress                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                             Mask                              |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                            nextHop                            |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                          destAddress                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                             Mask                              |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                            nextHop                            |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       :                                                               :
  //       :                               :
  //   ID is the car's ID of this routing table.
  struct Rm
  {
    struct Routing_Tuple{
      Ipv4Address destAddress, mask, nextHop;
    };
    
    uint32_t routingMessageSize;
    void SetRoutingMessageSize(uint32_t rms)
    {
      this->routingMessageSize = rms;
    }
    uint32_t GetRoutingMessageSize() const
    {
      return (this->routingMessageSize);
    }
    
    Ipv4Address ID;
    
    std::vector<Routing_Tuple> routingTables;
    

    void Print (std::ostream &os) const;
    uint32_t GetSerializedSize (void) const;
    void Serialize (Buffer::Iterator start) const;
    uint32_t Deserialize (Buffer::Iterator start, uint32_t messageSize);
  };

  //  Appointment Message Format
  //    One Appointment is for one car only.
  //    The proposed format of a appointment message is as follows:
  //
  //        0                   1                   2                   3
  //        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                             Car ID                            |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                       Appointment Type                        |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                 IpAddress of Next Forwarder                   |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+




 //  CARROUTEREQUEST_MESSAGE Format
  //    When a car cannot find a route to a destination 
  //    it will send a routingrequest message to lc as follows:
  //
  //        0                   1                   2                   3
  //        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                      sourceAddress                            |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                          destAddress                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                          tag                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       :                               :                               :
  //   ID is the car's ID 
  struct CRREQ
  {
    
    Ipv4Address sourceAddress,destAddress;
    uint32_t tag;
    void Print (std::ostream &os) const;
    uint32_t GetSerializedSize (void) const;
    void Serialize (Buffer::Iterator start) const;
    uint32_t Deserialize (Buffer::Iterator start, uint32_t messageSize);
  };
  //  CARROUTERESPONCE_MESSAGE Format
  //    When a car cannot find a route to a destination 
  //    it will send a routingrequest message to lc 
  //    lc will respond it as follows:
  //
  //        0                   1                   2                   3
  //        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                      sourceAddress                            |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                      destAddress                              |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                      transferAddress                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       :                                                               :
  //       :                               :                               :
  struct CRREP
  {
    
    Ipv4Address sourceAddress,destAddress,transferAddress;
    
    void Print (std::ostream &os) const;
    uint32_t GetSerializedSize (void) const;
    void Serialize (Buffer::Iterator start) const;
    uint32_t Deserialize (Buffer::Iterator start, uint32_t messageSize);
  };

 //  Aodv Routing Message Format
  //
  //    The proposed format of a aodv routing message is as follows:
  //
  //        0                   1                   2                   3
  //        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |               tag                     |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                      sourceAddress(ID)                        | //first send node id
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                          destAddress                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                             Mask                              |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                            jump numbers                       |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                          stablity                             |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //       |                        origintor                       |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //       |                        GPS      x                 |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      //       |                        GPS      y                 |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      //       |                        GPS      z                 |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   //       |                        direction                        |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+



  struct AodvRm
  {


	    uint32_t tag;
	    struct Position{
                        uint32_t X, Y, Z;
            };

	    Ipv4Address ID;//sourceAddress
	    Ipv4Address DesId;
	    Ipv4Mask mask;
	    uint32_t jump_nums;
	    float stability;
	    Ipv4Address Originator;
	    Position position;
	    enum direction dir;
    void SetPosition(double x, double y, double z)
    {
      this->position.X = IEEE754(x);
      this->position.Y = IEEE754(y);
      this->position.Z = IEEE754(z);
    }
    
    void GetPosition(double &x, double &y, double &z) const
    {
      x = rIEEE754(this->position.X);
      y = rIEEE754(this->position.Y);
      z = rIEEE754(this->position.Z);
    }
    
    Vector3D GetPosition() const
    {
      return Vector3D(rIEEE754(this->position.X),
                      rIEEE754(this->position.Y),
                      rIEEE754(this->position.Z));
    }


	    
	    void SetStability(float stab)
	    {
	      stability = IEEE754(stab);
	    }
	    float GetStability(void) const
	    {
	      return rIEEE754(stability);
	    }
	    //void Print (std::ostream &os) const;
	    uint32_t GetSerializedSize (void) const;
	    void Serialize (Buffer::Iterator start) const;
	    uint32_t Deserialize (Buffer::Iterator start, uint32_t messageSize);
  };



  //  Aodv reserse Routing Message Format
    //
    //    The proposed format of a aodv routing message is as follows:
    //
    //        0                   1                   2                   3
    //        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
    //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //       |                Aodv Routing Message Size                      |
    //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //       |                      sourceAddress(ID)                        | //first send node id
    //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //       |                          destAddress                          |
    //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   //       |                            first car'id                            |
   //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //       |                             Mask                              |
    //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
     //       |                             originator                             |
    //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //       |                             next                           |
    //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   


    struct Aodv_R_Rm
    {


  	    uint32_t routingMessageSize;
  	    void SetRoutingMessageSize(uint32_t rms)
  	    {
  	      this->routingMessageSize = rms;
  	    }
  	    uint32_t GetRoutingMessageSize() const
  	    {
  	      return (this->routingMessageSize);
  	    }

  	    Ipv4Address ID;//sourceAddress
  	    Ipv4Address DesId;
  	    Ipv4Address FirstCarId;
  	    Ipv4Mask mask;
  	     Ipv4Address originator;
  	     Ipv4Address next;
  	     direction next_dir;
  
  	    //void Print (std::ostream &os) const;
  	    uint32_t GetSerializedSize (void) const;
  	    void Serialize (Buffer::Iterator start) const;
  	    uint32_t Deserialize (Buffer::Iterator start, uint32_t messageSize);
    };



  //  Maintainment Message Format
  //    The proposed format of a Maintainment message is as follows:
  //
  //        0                   1                   2                   3
  //        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                             rORm                         |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //       |                             sourceID                            |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //       |                             sinkID                        |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                       transferID                  |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //       |                       dir                |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

  struct Maintainment
  {
    uint32_t rORm;//0 maintain , 1 rechose
    Ipv4Address sourceID;
    Ipv4Address sinkID;
    Ipv4Address transferID;
    enum direction dir;


    void Print (std::ostream &os) const;
    uint32_t GetSerializedSize (void) const;
    void Serialize (Buffer::Iterator start) const;
    uint32_t Deserialize (Buffer::Iterator start, uint32_t messageSize);
  };


private:
  struct
  {
    Hello hello;
    Rm rm;
    AodvRm aodvrm;
    Aodv_R_Rm aodv_r_rm;
    Maintainment mt;
    CRREQ crreq;
    CRREP crrep;
  } m_message; // union not allowed

public:

  Hello& GetHello ()
  {
    if (m_messageType == 0)
      {
        m_messageType = HELLO_MESSAGE;
      }
    else
      {
        NS_ASSERT (m_messageType == HELLO_MESSAGE);
      }
    return (m_message.hello);
  }

  Rm& GetRm ()
  {
    if (m_messageType == 0)
      {
        m_messageType = ROUTING_MESSAGE;
      }
    else
      {
        NS_ASSERT (m_messageType == ROUTING_MESSAGE);
      }
    return (m_message.rm);
  }

  Maintainment& GetMaintainment ()
  {
    if (m_messageType == 0)
      {
        m_messageType = MAINTAINMENT_MESSAGE;
      }
    else
      {
        NS_ASSERT (m_messageType == MAINTAINMENT_MESSAGE);
      }
    return (m_message.mt);
  }

  CRREQ& GetCRREQ ()
  {
    if (m_messageType == 0)
      {
        m_messageType = CARROUTEREQUEST_MESSAGE;
      }
    else
      {
        NS_ASSERT (m_messageType == CARROUTEREQUEST_MESSAGE);
      }
    return (m_message.crreq);
  }

  CRREP& GetCRREP ()
  {
    if (m_messageType == 0)
      {
        m_messageType = CARROUTERESPONCE_MESSAGE;
      }
    else
      {
        NS_ASSERT (m_messageType ==CARROUTERESPONCE_MESSAGE);
      }
    return (m_message.crrep);
  }

  AodvRm& GetAodvRm ()
  {
    if (m_messageType == 0)
      {
        m_messageType = AODV_ROUTING_MESSAGE;
      }
    else
      {
        NS_ASSERT (m_messageType == AODV_ROUTING_MESSAGE);
      }
    return (m_message.aodvrm);
  }

  Aodv_R_Rm& GetAodv_R_Rm ()
  {
    if (m_messageType == 0)
      {
        m_messageType = AODV_REVERSE_MESSAGE;
      }
    else
      {
        NS_ASSERT (m_messageType == AODV_REVERSE_MESSAGE);
      }
    return (m_message.aodv_r_rm);
  }

  const Hello& GetHello () const
  {
    NS_ASSERT (m_messageType == HELLO_MESSAGE);
    return (m_message.hello);
  }

  const Rm& GetRm () const
  {
    NS_ASSERT (m_messageType == ROUTING_MESSAGE);
    return (m_message.rm);
  }

 const AodvRm& GetAodvRm () const
  {
    NS_ASSERT (m_messageType == AODV_ROUTING_MESSAGE);
    return (m_message.aodvrm);
  }

 const Aodv_R_Rm& GetAodv_R_Rm() const
  {
	 //std::cout<<"m_messagetype"<<m_messageType<<std::endl;
    NS_ASSERT (m_messageType == AODV_REVERSE_MESSAGE);
    return (m_message.aodv_r_rm);
  }
  const Maintainment& GetMaintainment () const
  {
    NS_ASSERT (m_messageType == MAINTAINMENT_MESSAGE);
    return (m_message.mt);
  }


  const CRREQ& GetCRREQ () const
  {
    NS_ASSERT (m_messageType == CARROUTEREQUEST_MESSAGE);
    return (m_message.crreq);
  }

  const CRREP& GetCRREP () const
  {
    NS_ASSERT (m_messageType == CARROUTERESPONCE_MESSAGE);
    return (m_message.crrep);
  }
};

static inline std::ostream& operator<< (std::ostream& os, const PacketHeader & packet)
{
  packet.Print (os);
  return (os);
}

static inline std::ostream& operator<< (std::ostream& os, const MessageHeader & message)
{
  message.Print (os);
  return (os);
}

typedef std::vector<MessageHeader> MessageList;

static inline std::ostream& operator<< (std::ostream& os, const MessageList & messages)
{
  os << "[";
  for (std::vector<MessageHeader>::const_iterator messageIter = messages.begin ();
       messageIter != messages.end (); messageIter++)
    {
      messageIter->Print (os);
      if (messageIter+1 != messages.end ())
        os << ", ";
    }
  os << "]";
  return (os);
}


}//namespace sdn
}//namespace ns3


#endif /* SDN_HEADER_H */
