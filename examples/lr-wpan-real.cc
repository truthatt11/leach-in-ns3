/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 The Boeing Company
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
 * Author:  Tom Henderson <thomas.r.henderson@boeing.com>
 */

/*
 * Try to send data end-to-end through a LrWpanMac <-> LrWpanPhy <->
 * SpectrumChannel <-> LrWpanPhy <-> LrWpanMac chain
 *
 * Trace Phy state changes, and Mac DataIndication and DataConfirm events
 * to stdout
 */
#include <ns3/log.h>
#include <ns3/core-module.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/internet-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/ipv4-interface-container.h>
#include <ns3/packet.h>
#include <ns3/wifi-module.h>
#include <ns3/socket.h>

#include <iostream>
#include <cstdio>
#include <sstream>
#include <string>
#include <iomanip>
#include <cstdlib>

using namespace ns3;
using namespace std;



NodeContainer nodes, gw;
Ptr<LrWpanNetDevice> devices[20];
Ptr<Socket> gw_socket[4], sink_socket;
Ptr<Node> sink;
NetDeviceContainer devs, sink_dev;
Ipv4InterfaceContainer sink_inf, devs_inf;

static void DataIndication (McpsDataIndicationParams params, Ptr<Packet> p)
{
  uint8_t dst[2];
  NS_LOG_UNCOND ("Received packet of size " << p->GetSize () << ", Src: " << params.m_srcAddr << ", Dst: " << params.m_dstAddr);
  Ptr<Packet> a = Create<Packet> (*p);
  params.m_dstAddr.CopyTo(dst);
//  NS_LOG_UNCOND ((int)dst[0] << " " << (int)dst[1]);
  gw_socket[dst[1]-17]->Send(a);
  NS_LOG_UNCOND ("Forwarded to the sink");
}

static void DataConfirm (McpsDataConfirmParams params)
{
  NS_LOG_UNCOND ("LrWpanMcpsDataConfirmStatus = " << params.m_status);
}



class iotTest{
public:
  void Run();
  void setup();

private:
  void ReceivePacket (Ptr <Socket> socket);
};



int main (int argc, char *argv[])
{
  bool verbose = false;
  iotTest inst;
  CommandLine cmd;

  cmd.AddValue ("verbose", "turn on all log components", verbose);

  cmd.Parse (argc, argv);

  LrWpanHelper lrWpanHelper;
  if (verbose)
    {
      lrWpanHelper.EnableLogComponents ();
    }

  // Enable calculation of FCS in the trailers. Only necessary when interacting with real devices or wireshark.
  // GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  inst.setup();
  inst.Run();


  return 0;
}

void
iotTest::ReceivePacket (Ptr <Socket> socket)
{
  NS_LOG_UNCOND("Sink received one pacekt");
}

void
iotTest::setup ()
{
  nodes.Create(16);
  gw.Create(4);
  sink = CreateObject <Node> ();

  // Each device must be attached to the same channel
  Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
  Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel> ();
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  channel->AddPropagationLossModel (propModel);
  channel->SetPropagationDelayModel (delayModel);
  
  // In a for loop
  string a, b;
  for (int i=0; i < 16; i++) {
    devices[i] = CreateObject<LrWpanNetDevice> ();
    Ptr<ConstantPositionMobilityModel> senderMobility = CreateObject<ConstantPositionMobilityModel> ();
    senderMobility->SetPosition (Vector ((i/4)*20,(i%4)*20,0));
    
    stringstream ss;
    ss << setbase(16) << setfill('0') << setw(2);
    ss << i+1;
    ss >> a;
    b = "00:"+a;
    devices[i]->SetAddress (Mac16Address (b.c_str()));
    devices[i]->SetChannel (channel);
    
    nodes.Get(i)->AddDevice (devices[i]);
    devices[i]->GetPhy ()->SetMobility (senderMobility);
    
    McpsDataConfirmCallback cb0;
    cb0 = MakeCallback (&DataConfirm);
    devices[i]->GetMac ()->SetMcpsDataConfirmCallback (cb0);
    
    McpsDataIndicationCallback cb1;
    cb1 = MakeCallback (&DataIndication);
    devices[i]->GetMac ()->SetMcpsDataIndicationCallback (cb1);
  }
  
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue("DsssRate11Mbps"), "ControlMode", StringValue("DsssRate11Mbps"));
  
  for (int i=0; i < 4; i++) {
    devices[i+16] = CreateObject<LrWpanNetDevice> ();
    Ptr<ConstantPositionMobilityModel> gatewayMobility = CreateObject<ConstantPositionMobilityModel> ();
    gatewayMobility->SetPosition (Vector ((i/2)*40+10,(i%2)*40+10,0));
    
    stringstream ss;
    ss << setbase(16) << setfill('0') << setw(2);
    ss << i+17;
    ss >> a;
    b = "00:"+a;
    devices[i+16]->SetAddress (Mac16Address (b.c_str()));
    devices[i+16]->SetChannel (channel);
    
    gw.Get(i)->AddDevice (devices[i+16]);
    NetDeviceContainer a = wifi.Install (wifiPhy, wifiMac, gw.Get(i));
    devices[i+16]->GetPhy ()->SetMobility (gatewayMobility);
    Ptr<WifiNetDevice> netdev = DynamicCast<WifiNetDevice>(a.Get(0));
    Ptr<WifiPhy> phy = DynamicCast<WifiPhy>(netdev->GetPhy ());
    phy->SetMobility (gatewayMobility);
    devs.Add(a);
    
    McpsDataConfirmCallback cb0;
    cb0 = MakeCallback (&DataConfirm);
    devices[i+16]->GetMac ()->SetMcpsDataConfirmCallback (cb0);
    
    McpsDataIndicationCallback cb1;
    cb1 = MakeCallback (&DataIndication);
    devices[i+16]->GetMac ()->SetMcpsDataIndicationCallback (cb1);
  }

  Ptr<ConstantPositionMobilityModel> sinkMobility = CreateObject<ConstantPositionMobilityModel> ();
  sinkMobility->SetPosition (Vector (100,100,0));
  sink_dev = wifi.Install (wifiPhy, wifiMac, sink);
  Ptr<WifiNetDevice> netdev = DynamicCast<WifiNetDevice>(sink_dev.Get(0));
  Ptr<WifiPhy> phy = DynamicCast<WifiPhy>(netdev->GetPhy ());
  phy->SetMobility (sinkMobility);
  
  wifiPhy.EnablePcapAll ("IoT-Test");
	
  InternetStackHelper stack;
  stack.Install (gw);
  stack.Install (sink);
	
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  devs_inf = address.Assign (devs);
  sink_inf = address.Assign (sink_dev);
  
  sink_socket = Socket::CreateSocket (sink, UdpSocketFactory::GetTypeId ());
  InetSocketAddress local = InetSocketAddress (sink_inf.GetAddress(0,0), 9);
  sink_socket->Bind(local);
  sink_socket->SetRecvCallback (MakeCallback ( &iotTest::ReceivePacket, this));
  
  for(int i=0; i<4; i++) {
    gw_socket[i] = Socket::CreateSocket (gw.Get(i), UdpSocketFactory::GetTypeId ());
    gw_socket[i]->Bind();
//    NS_LOG_UNCOND("sink address: " << InetSocketAddress(sink_inf.GetAddress(0,0), 9));
    gw_socket[i]->Connect(InetSocketAddress(sink_inf.GetAddress(0,0), 9));
    gw_socket[i]->SetAllowBroadcast (true);
  }
  
  return;
}

void
iotTest::Run ()
{	
  double schedule_time = 0.2;
  // Every sensor node transmit data to their gateway
  for(int i=0; i<4; i++) {
    McpsDataRequestParams params;
    params.m_srcAddrMode = SHORT_ADDR;
    params.m_dstAddrMode = SHORT_ADDR;
    params.m_dstPanId = 0;
    params.m_dstAddr = devices[i+16]->GetMac ()->GetShortAddress ();
    params.m_msduHandle = 0;
    params.m_txOptions = TX_OPTION_ACK;
    
    for(int j=0; j<4; j++) {
      Ptr<Packet> p0 = Create<Packet> (50);  // 50 bytes of dummy data
	  Ptr<LrWpanNetDevice> dev1 = DynamicCast<LrWpanNetDevice> (nodes.Get ((i/2)*8+(i%2)*2+(j%2)+(j/2)*4)->GetDevice (0));
      Simulator::Schedule (Seconds (schedule_time), &LrWpanMac::McpsDataRequest, dev1->GetMac (), params, p0);
      schedule_time += 0.2;
	}
  }

  Simulator::Stop (Seconds (20.0));
  Simulator::Run ();

  Simulator::Destroy ();
  return ;
}
