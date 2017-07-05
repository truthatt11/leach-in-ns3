/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 Hemanth Narra, Yufei Cheng
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
 * Author: Yufei Cheng   <yfcheng@ittc.ku.edu>
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

#ifndef LEACH_ROUTING_PROTOCOL_H
#define LEACH_ROUTING_PROTOCOL_H

#include "leach-rtable.h"
#include "leach-packet-queue.h"
#include "leach-packet.h"
#include "ns3/node.h"
#include "ns3/random-variable-stream.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/vector.h"
#include <vector>

namespace ns3 {
namespace leach {

/**
 * \ingroup leach
 * \brief LEACH routing protocol.
 */
class RoutingProtocol : public Ipv4RoutingProtocol
{
public:
  static TypeId
  GetTypeId (void);
  static const uint32_t LEACH_PORT;

  /// c-tor
  RoutingProtocol ();
  virtual
  ~RoutingProtocol ();
  virtual void
  DoDispose ();

  // From Ipv4RoutingProtocol
  Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif, Socket::SocketErrno &sockerr);
  bool RouteInput (Ptr<const Packet> p, const Ipv4Header &header, Ptr<const NetDevice> idev, UnicastForwardCallback ucb,
                   MulticastForwardCallback mcb, LocalDeliverCallback lcb, ErrorCallback ecb);
  virtual void PrintRoutingTable (Ptr<OutputStreamWrapper> stream) const;
  virtual void NotifyInterfaceUp (uint32_t interface);
  virtual void NotifyInterfaceDown (uint32_t interface);
  virtual void NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address);
  virtual void NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address);
  virtual void SetIpv4 (Ptr<Ipv4> ipv4);
  
  // Methods to handle protocol parameters
  void SetPosition (Vector f);
  Vector GetPosition () const;
  uint32_t GetDropped() const;

 /**
  * Assign a fixed random variable stream number to the random variables
  * used by this model.  Return the number of streams (possibly zero) that
  * have been assigned.
  *
  * \param stream first stream index to use
  * \return the number of stream indices assigned by this model
  */
  int64_t AssignStreams (int64_t stream);

private:
  
  // Protocol parameters.
  /// Holdtimes is the multiplicative factor of PeriodicUpdateInterval for which the node waits since the last update
  /// before flushing a route from the routing table. If PeriodicUpdateInterval is 8s and Holdtimes is 3, the node
  /// waits for 24s since the last update to flush this route from its routing table.
  uint32_t Round;
  uint32_t valid;
  uint32_t cluster_head_this_round;
  uint32_t isSink;
  uint32_t m_dropped;

  /// PeriodicUpdateInterval specifies the periodic time interval between which the a node broadcasts
  /// its entire routing table.
  Time m_periodicUpdateInterval;
  /// Nodes IP address
  Ipv4Address m_mainAddress;
  /// Cluster Head/Sink Address
  Ipv4Address m_targetAddress;
  /// The ultimate sink
  Ipv4Address m_sinkAddress;
  /// the closest distance node
  double m_dist;
  /// cluster member list
  std::vector<Ipv4Address> m_clusterMember;
  /// IP protocol
  Ptr<Ipv4> m_ipv4;
  /// Raw socket per each IP interface, map socket -> iface address (IP + mask)
  std::map<Ptr<Socket>, Ipv4InterfaceAddress> m_socketAddresses;
  /// Loopback device used to defer route requests until a route is found
  Ptr<NetDevice> m_lo;
  /// Main Routing table for the node
  RoutingTable m_routingTable;
  /// From selecting CHs, the best stores here
  RoutingTableEntry m_bestRoute;
  /// The maximum number of packets that we allow a routing protocol to buffer.
  uint32_t m_maxQueueLen;
  /// Node position
  Vector m_position;
  /// A "drop front on full" queue used by the routing layer to buffer packets to which it does not have a route.
  PacketQueue m_queue;
  /// Unicast callback for own packets
  UnicastForwardCallback m_scb;
  /// Error callback for own packets
  ErrorCallback m_ecb;

private:
  
  /// Start protocol operation
  void
  Start ();
  /// Queue packet until we find a route
  void
  EnqueuePacket (Ptr<Packet> p, const Ipv4Header & header);
  /// Decide whether to send the packets in the buffer
  bool
  DataAggregation (Ptr<Packet> p);
  /// De-aggregate chunk of data
  bool
  DeAggregate (Ptr<Packet> in, Ptr<Packet>& out);
  
  /// Find socket with local interface address iface
  Ptr<Socket>
  FindSocketWithInterfaceAddress (Ipv4InterfaceAddress iface) const;
  /// Find socket with local address iface
  Ptr<Socket>
  FindSocketWithAddress (Ipv4Address iface) const;
  
  // Receive leach control packets
  /// Receive and process leach control packet
  void
  RecvLeach (Ptr<Socket> socket);

  void
  Send (Ptr<Ipv4Route>, Ptr<const Packet>, const Ipv4Header &);
  /// Create loopback route for given header
  Ptr<Ipv4Route>
  LoopbackRoute (const Ipv4Header & header, Ptr<NetDevice> oif) const;
  /// Triggered by a timer, this broadcast is done after 1 second since the selection of CH
  void
  SendBroadcast ();
  /// Select the cluster head selection result
  void
  PeriodicUpdate ();
  /// Cluster member tell their cluster head
  void
  RespondToClusterHead ();
  /// Notify that packet is dropped for some reason
  void
  Drop (Ptr<const Packet>, const Ipv4Header &, Socket::SocketErrno);
  /// Timer to trigger periodic updates from a node
  Timer m_periodicUpdateTimer;
  /// Timer used by the trigger updates in case of Weighted Settling Time is used
  Timer m_broadcastClusterHeadTimer;
  /// Timer to feedback the cluster head its member
  Timer m_respondToClusterHeadTimer;

  /// Provides uniform random variables.
  Ptr<UniformRandomVariable> m_uniformRandomVariable;  
};

}
}

#endif /* LEACH_ROUTING_PROTOCOL_H */
