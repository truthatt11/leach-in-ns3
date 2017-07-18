/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 Hemanth Narra
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
 * Author: Hemanth Narra <hemanth@ittc.ku.com>
 *
 * James P.G. Sterbenz <jpgs@ittc.ku.edu>, director
 * ResiliNets Research Group  http://wiki.ittc.ku.edu/resilinets
 * Information and Telecommunication Technology Center (ITTC)
 * and Department of Electrical Engineering and Computer Science
 * The University of Kansas Lawrence, KS USA.
 *
 * Work supported in part by NSF FIND (Future Internet Design) Program
 * under grant CNS-0626918 (Postmodern Internet Architecture),
 * NSF grant CNS-1050226 (Multilayer Network Resilience Analysis and Experimentation on GENI),
 * US Department of Defense (DoD), and ITTC at The University of Kansas.
 */

#ifndef LEACH_PACKETQUEUE_H
#define LEACH_PACKETQUEUE_H

#include <vector>
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/simulator.h"
#include "ns3/leach-packet.h"
//#include "ns3/udp-header.h"

#include <iostream>

namespace ns3 {
namespace leach {
/**
 * \ingroup leach
 * \brief LEACH Queue Entry
 */
class QueueEntry
{
public:
  typedef Ipv4RoutingProtocol::UnicastForwardCallback UnicastForwardCallback;
  /// c-tor
  QueueEntry (Ptr<Packet> pa = 0, Ipv4Header const & h = Ipv4Header ())
    : m_packet (pa),
      m_header (h)
  {
    if(pa != 0) {
      Packet a (*pa);
//      std::cout << "QueueEntry constructor\n";
//      pa->Print(std::cout);
      LeachHeader hdr;
      a.RemoveHeader(hdr);
      m_deadline = hdr.GetDeadline();
    }
  }

  /**
   * Compare queue entries
   * \return true if equal
   */
  bool operator== (QueueEntry const & o) const
  {
    return ((m_packet == o.m_packet) && (m_header.GetDestination () == o.m_header.GetDestination ()));
  }
  
  // Fields
  Ptr<Packet> GetPacket () const
  {
    return m_packet;
  }
  void SetPacket (Ptr<Packet> p)
  {
    m_packet = p;
  }
  Ipv4Header GetIpv4Header () const
  {
    return m_header;
  }
  void SetIpv4Header (Ipv4Header h)
  {
    m_header = h;
  }
  Time GetDeadline () const
  {
    return m_deadline;
  }
  void SetDeadline (Time d)
  {
    m_deadline = d;
  }

private:
  /// Data packet
  Ptr<Packet> m_packet;
  /// IP header
  Ipv4Header m_header;
  /// Deadline
  Time m_deadline;
};

/**
 * \ingroup leach
 * \brief LEACH Packet queue
 *
 * When a route is not available, the packets are queued. Every node can buffer up to 5 packets per
 * destination. We have implemented a "drop front on full" queue where the first queued packet will be dropped
 * to accommodate newer packets.
 */
class PacketQueue
{
public:
  /// Default c-tor
  PacketQueue ()
  {
  }
  /// Push entry in queue, if there is no entry with the same packet and destination address in queue.
  bool Enqueue (QueueEntry & entry);
  /// Return first found (the earliest) entry for given destination
  bool Dequeue (Ipv4Address dst, QueueEntry & entry);
  /// Remove all packets with destination IP address dst
//  void DropPacketWithDst (Ipv4Address dst);
  /// Finds whether a packet with destination dst exists in the queue
  bool Find (Ipv4Address dst);
  /// Drop the idx-th entry
  void Drop (uint32_t idx);
  /// Get count of packets with destination dst in the queue
  uint32_t GetCountForPacketsWithDst (Ipv4Address dst);
  /// Number of entries
  uint32_t GetSize ();
  
  // Fields
  Time GetQueueTimeout () const
  {
    return m_queueTimeout;
  }
  void SetQueueTimeout (Time t)
  {
    m_queueTimeout = t;
  }
  QueueEntry& operator[] (size_t idx)
  {
    return m_queue[idx];
  }

private:
  std::vector<QueueEntry> m_queue;
  /// The maximum period of time that a routing protocol is allowed to buffer a packet for, seconds.
  Time m_queueTimeout;
  static bool IsEqual (QueueEntry en, const Ipv4Address dst)
  {
    return (en.GetIpv4Header ().GetDestination () == dst);
  }
};
}
}
#endif /* LEACH_PACKETQUEUE_H */
