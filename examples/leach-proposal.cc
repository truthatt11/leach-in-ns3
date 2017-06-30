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
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/internet-module.h"
#include "ns3/leach-helper.h"
#include "ns3/wsn-helper.h"
#include "ns3/wifi-module.h"
#include "ns3/energy-module.h"
#include "ns3/vector.h"
#include "ns3/leach-packet.h"
#include "ns3/udp-header.h"
#include <iostream>
#include <cmath>


using namespace ns3;

uint16_t port = 9;
uint32_t packetsGenerated = 0;

NS_LOG_COMPONENT_DEFINE ("LeachProposal");


/// Trace function for remaining energy at node.
void
RemainingEnergy (double oldValue, double remainingEnergy)
{
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds ()
                 << "s Current remaining energy = " << remainingEnergy << "J");
}

/// Trace function for total energy consumption at node.
void
TotalEnergy (double oldValue, double totalEnergy)
{
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds ()
                 << "s Total energy consumed by radio = " << totalEnergy << "J");
}

/// record packet counts
void
TotalPackets (uint32_t oldValue, uint32_t newValue)
{
  packetsGenerated += (newValue - oldValue);
}


class LeachProposal
{
public:
  LeachProposal ();
  void CaseRun (uint32_t nWifis,
                uint32_t nSinks,
                double totalTime,
                std::string rate,
                std::string phyMode,
                uint32_t periodicUpdateInterval,
                double dataStart);

private:
  uint32_t m_nWifis;
  uint32_t m_nSinks;
  double m_totalTime;
  std::string m_rate;
  std::string m_phyMode;
  uint32_t m_periodicUpdateInterval;
  double m_dataStart;
  uint32_t bytesTotal;
  uint32_t packetsReceived;
  uint32_t packetsDecompressed;
  Vector positions[105];

  NodeContainer nodes;
  NetDeviceContainer devices;
  Ipv4InterfaceContainer interfaces;
  
  EnergySourceContainer sources;

private:
  void CreateNodes ();
  void CreateDevices ();
  void InstallInternetStack (std::string tr_name);
  void InstallApplications ();
  void SetupMobility ();
  void SetupEnergyModel ();
  void ReceivePacket (Ptr <Socket> );
  Ptr <Socket> SetupPacketReceive (Ipv4Address, Ptr <Node> );

};

int main (int argc, char **argv)
{
  LeachProposal test;
  uint32_t nWifis = 30;
  uint32_t nSinks = 1;
  double totalTime = 50.0;
//  double totalTime = 20.0;
  std::string rate ("8kbps");
  std::string phyMode ("DsssRate11Mbps");
  uint32_t periodicUpdateInterval = 15;
  double dataStart = 20.0;

  CommandLine cmd;
  cmd.AddValue ("nWifis", "Number of WiFi nodes[Default:30]", nWifis);
  cmd.AddValue ("nSinks", "Number of WiFi sink nodes[Default:1]", nSinks);
  cmd.AddValue ("totalTime", "Total Simulation time[Default:50]", totalTime);
  cmd.AddValue ("phyMode", "Wifi Phy mode[Default:DsssRate11Mbps]", phyMode);
  cmd.AddValue ("rate", "CBR traffic rate[Default:8kbps]", rate);
  cmd.AddValue ("periodicUpdateInterval", "Periodic Interval Time[Default=15]", periodicUpdateInterval);
  cmd.AddValue ("dataStart", "Time at which nodes start to transmit data[Default=20.0]", dataStart);
  cmd.Parse (argc, argv);

  SeedManager::SetSeed (12345);

  Config::SetDefault ("ns3::WsnApplication::PacketSize", StringValue ("64"));
  Config::SetDefault ("ns3::WsnApplication::DataRate", StringValue (rate));
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (phyMode));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2000"));

  test = LeachProposal ();
  test.CaseRun (nWifis, nSinks, totalTime, rate, phyMode, periodicUpdateInterval, dataStart);
  
  return 0;
}

LeachProposal::LeachProposal ()
  : bytesTotal (0),
    packetsReceived (0),
    packetsDecompressed (0)
{
}

void
LeachProposal::ReceivePacket (Ptr <Socket> socket)
{
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds () << " Received one packet!");
  Ptr <Packet> packet;
  
  while ((packet = socket->Recv ()))
    {
      leach::LeachHeader leachHeader;
      UdpHeader udpHeader;
      
      bytesTotal += packet->GetSize();
      packet->RemoveHeader(leachHeader);
      if(leachHeader.GetDeadline() > Simulator::Now()) packetsDecompressed++;
      packet->RemoveAtStart(16);

      while(packet->GetSize()>0) {
        packet->RemoveHeader(udpHeader);
        packet->RemoveHeader(leachHeader);
        packet->RemoveAtStart(16);
        
        if(leachHeader.GetDeadline() > Simulator::Now()) packetsDecompressed++;
      }
      packetsReceived++;
    }
}

Ptr <Socket>
LeachProposal::SetupPacketReceive (Ipv4Address addr, Ptr <Node> node)
{

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr <Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, port);
  
  NS_LOG_INFO(addr);
  
  sink->Bind (local);
  sink->SetRecvCallback (MakeCallback ( &LeachProposal::ReceivePacket, this));

  return sink;
}

void
LeachProposal::CaseRun (uint32_t nWifis, uint32_t nSinks, double totalTime, std::string rate,
                           std::string phyMode, uint32_t periodicUpdateInterval, double dataStart)
{
  m_nWifis = nWifis;
  m_nSinks = nSinks;
  m_totalTime = totalTime;
  m_rate = rate;
  m_phyMode = phyMode;
  m_periodicUpdateInterval = periodicUpdateInterval;
  m_dataStart = dataStart;

  std::stringstream ss;
  ss << m_nWifis;
  std::string t_nodes = ss.str ();

  std::stringstream ss3;
  ss3 << m_totalTime;
  std::string sTotalTime = ss3.str ();

  std::string tr_name = "Leach_Manet_" + t_nodes + "Nodes_" + sTotalTime + "SimTime";
//  std::cout << "Trace file generated is " << tr_name << ".tr\n";

  CreateNodes ();
  CreateDevices ();
  SetupMobility ();
  SetupEnergyModel();
  InstallInternetStack (tr_name);
  InstallApplications ();

  std::cout << "\nStarting simulation for " << m_totalTime << " s ...\n";

//  CheckThroughput ();

  Simulator::Stop (Seconds (m_totalTime));
  Simulator::Run ();

  NS_LOG_UNCOND ("Total bytes received: " << bytesTotal);
  NS_LOG_UNCOND ("Total packets received/decompressed/generated: " << packetsReceived << "/" << packetsDecompressed << "/" << packetsGenerated);
  for (uint32_t i=0; i<m_nWifis; i++)
    {
      Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources.Get (i));
      Ptr<DeviceEnergyModel> basicRadioModelPtr = basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
      Ptr<WifiRadioEnergyModel> ptr = DynamicCast<WifiRadioEnergyModel> (basicRadioModelPtr);
      NS_ASSERT (basicRadioModelPtr != NULL);
      NS_LOG_UNCOND("Idle time: " << ptr->GetIdleTime() << ", Tx Time: " << ptr->GetTxTime() << ", Rx Time: " << ptr->GetRxTime());
    }

  Simulator::Destroy ();
}

void
LeachProposal::CreateNodes ()
{
  std::cout << "Creating " << (unsigned) m_nWifis << " nodes.\n";
  nodes.Create (m_nWifis);
  NS_ASSERT_MSG (m_nWifis > m_nSinks, "Sinks must be less or equal to the number of nodes in network");
}

void
LeachProposal::SetupMobility ()
{
  MobilityHelper mobility;
  ObjectFactory pos;
  uint32_t count = 0;
  pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
  pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));

  Ptr <PositionAllocator> taPositionAlloc = pos.Create ()->GetObject <PositionAllocator> ();
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (taPositionAlloc);
  mobility.Install (nodes);
  
  for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i)
    {
      Ptr<MobilityModel> model = (*i)->GetObject<MobilityModel> ();
      positions[count++] = model->GetPosition();
    }
}

void
LeachProposal::SetupEnergyModel()
{
  /** Energy Model **/
  /***************************************************************************/
  /* energy source */
  BasicEnergySourceHelper basicSourceHelper;
  // configure energy source
  basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (100));
  // install source
  /*EnergySourceContainer */sources = basicSourceHelper.Install (nodes);
  /* device energy model */
  WifiRadioEnergyModelHelper radioEnergyHelper;
  // install device model
  DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (devices, sources);
  /***************************************************************************/
  
  /*
  for (uint32_t i=0; i<m_nWifis; i++)
    {
      Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources.Get (i));
      basicSourcePtr->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergy));
      // device energy model
      Ptr<DeviceEnergyModel> basicRadioModelPtr =
        basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
      NS_ASSERT (basicRadioModelPtr != NULL);
      basicRadioModelPtr->TraceConnectWithoutContext ("TotalEnergyConsumption", MakeCallback (&TotalEnergy));
    }
  */
}

void
LeachProposal::CreateDevices ()
{
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue (m_phyMode), "ControlMode",
                                StringValue (m_phyMode));
  devices = wifi.Install (wifiPhy, wifiMac, nodes);

  AsciiTraceHelper ascii;
  wifiPhy.EnablePcapAll ("Leach-Manet");
}

void
LeachProposal::InstallInternetStack (std::string tr_name)
{
  LeachHelper leach;
  leach.Set ("PeriodicUpdateInterval", TimeValue (Seconds (m_periodicUpdateInterval)));
  InternetStackHelper stack;
  uint32_t count = 0;

  for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i)
    {
      leach.Set("Position", Vector3DValue(positions[count++]));
      stack.SetRoutingHelper (leach); // has effect on the next Install ()
      stack.Install (*i);
    }
  //stack.Install (nodes);        // should give change to leach protocol on the position property
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  interfaces = address.Assign (devices);
}

void
LeachProposal::InstallApplications ()
{
  Ptr<Node> node = NodeList::GetNode (0);
  Ipv4Address nodeAddress = node->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
  Ptr<Socket> sink = SetupPacketReceive (nodeAddress, node);
  
  WsnHelper wsn1 ("ns3::UdpSocketFactory", Address (InetSocketAddress (interfaces.GetAddress (0), port)));
  wsn1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  wsn1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  wsn1.SetAttribute ("PacketDeadlineLen", IntegerValue(3));  // default
  wsn1.SetAttribute ("PacketDeadlineMin", IntegerValue(5));  // default
  
  for (uint32_t clientNode = 1; clientNode <= m_nWifis - 1; clientNode++ )
    {
      ApplicationContainer apps1 = wsn1.Install (nodes.Get (clientNode));
      Ptr<WsnApplication> wsnapp = DynamicCast<WsnApplication> (apps1.Get (0));
      Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
      
      apps1.Start (Seconds (var->GetValue (m_dataStart, m_dataStart + 1)));
      apps1.Stop (Seconds (m_totalTime));
      wsnapp->TraceConnectWithoutContext ("PktCount", MakeCallback (&TotalPackets));
    }
}
