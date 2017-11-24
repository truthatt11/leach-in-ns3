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
#include <ns3/network-module.h>
#include <ns3/lr-wpan-module.h>
#include "ns3/mobility-module.h"
#include <ns3/internet-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/ipv4-interface-container.h>
#include <ns3/packet.h>
#include <ns3/wifi-module.h>
#include "ns3/energy-module.h"
#include <ns3/socket.h>
#include "ns3/netanim-module.h"

#include "ns3/leach-packet.h"

#include <iostream>
#include <cstdio>
#include <sstream>
#include <string>
#include <iomanip>
#include <cstdlib>
#include <vector>

#define DA_PROP
//#define DA_OPT
//#define DA_CL



using namespace ns3;
using namespace std;


static bool AggregationPolicy (int);
static void Aggregate (Ptr<Packet>, int);
static void Enqueue (Ptr<Packet>, int);

#ifdef DA_PROP
static bool Proposal (int);
#endif
#ifdef DA_OPT
static bool OptTM (int);
#endif
#ifdef DA_CL
static bool ControlLimit (int);
#endif

static NodeContainer nodes, gw;
static Ptr<LrWpanNetDevice> devices[68];
static Ptr<Socket> gw_socket[4], sink_socket;
static Ptr<Node> sink;
static NetDeviceContainer devs, sink_dev;
static Ipv4InterfaceContainer sink_inf, devs_inf;
static uint32_t total_packet = 0, measurementCount = 0;
static uint32_t m_dropped = 0, total_gened = 0, gened[68];
static vector<Ptr<Packet>> gw_buffer[4];
static int m_lambda = 2;
static Time rx_time[70], tx_time[70], from_time[70];
static int periodic = 0;
EnergySourceContainer sources;



static void DataIndication (McpsDataIndicationParams params, Ptr<Packet> p)
{
  uint8_t dst[2];
//  NS_LOG_UNCOND ("Received packet of size " << p->GetSize () << ", Src: " << params.m_srcAddr << ", Dst: " << params.m_dstAddr);
  Ptr<Packet> a = Create<Packet> (*p);
  params.m_dstAddr.CopyTo(dst);
	
  // Instead of TX, enqueue -> data aggregation -> Tx or not
  int gw_index = dst[1]-65;
  if(AggregationPolicy(gw_index)) {
    Aggregate (a, gw_index);
    gw_socket[gw_index]->Send(a);
//    NS_LOG_UNCOND ("Forwarded to the sink");
  }else {
    Enqueue (a, gw_index);
  }
}

static void DataConfirm (McpsDataConfirmParams params)
{
//  NS_LOG_UNCOND ("LrWpanMcpsDataConfirmStatus = " << params.m_status);
}

static void StateChangeNotification (std::string context, Time now, LrWpanPhyEnumeration oldState, LrWpanPhyEnumeration newState)
{
  // oldState = RX_ON, newState = BUSY_RX  ->  RX start
  // newState = BUSY_RX, newState = RX_ON  ->  RX end
  // oldState = TX_ON, newState = BUSY_TX  ->  TX start
  // oldState = BUSY_TX, newState = TX_ON  ->  TX end
  int index = 0;
	
  for (unsigned i=0; i<context.size(); i++) {
    index = index*10 + (context[i]-'0');
  }
//  NS_LOG_UNCOND (index << "/" << context);
	
  if (oldState == LrWpanPhyEnumeration::IEEE_802_15_4_PHY_RX_ON &&
      newState == LrWpanPhyEnumeration::IEEE_802_15_4_PHY_BUSY_RX)
  {
    from_time[index] = now;
  }
  if (oldState == LrWpanPhyEnumeration::IEEE_802_15_4_PHY_BUSY_RX &&
      newState == LrWpanPhyEnumeration::IEEE_802_15_4_PHY_RX_ON)
  {
    rx_time[index] += now - from_time[index];
  }
  if (oldState == LrWpanPhyEnumeration::IEEE_802_15_4_PHY_TX_ON &&
      newState == LrWpanPhyEnumeration::IEEE_802_15_4_PHY_BUSY_TX)
  {
    from_time[index] = now;
  }
  if (oldState == LrWpanPhyEnumeration::IEEE_802_15_4_PHY_BUSY_TX &&
      newState == LrWpanPhyEnumeration::IEEE_802_15_4_PHY_TX_ON)
  {
    tx_time[index] += now - from_time[index];
  }
}

double max(double a, double b) {
    return (a>b)?a:b;
}

class iotTest{
public:
  void Run();
  void setup();
  void setMobility (Ptr<Node>, Vector);
  void TxPkt(int);
  void ScheduleNext (int);

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
//  NS_LOG_UNCOND("Sink received one packet");
  Ptr <Packet> packet;
	
  while (packet = socket->Recv()) {
    leach::LeachHeader hdr;
	  
    while(packet->GetSize()) {
      packet->RemoveHeader(hdr);
      packet->RemoveAtStart(17);
		
      if(hdr.GetDeadline() < Simulator::Now()) m_dropped++;
      else measurementCount++;
    }
  }
  total_packet++;
}

void
iotTest::setup ()
{
  nodes.Create(64);
  gw.Create(4);
  sink = CreateObject <Node> ();

  // Each device must be attached to the same channel
  Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
  Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel> ();
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  channel->AddPropagationLossModel (propModel);
  channel->SetPropagationDelayModel (delayModel);
  
  // sensor node lr-wpan interface
  string a, b;
  for (int i=0; i < 64; i++) {
    char name[5];
    devices[i] = CreateObject<LrWpanNetDevice> ();
    
    stringstream ss;
    ss << setbase(16) << setfill('0') << setw(2);
    ss << i+1;
    ss >> a;
    b = "00:"+a;
    devices[i]->SetAddress (Mac16Address (b.c_str()));
    devices[i]->SetChannel (channel);
    
    nodes.Get(i)->AddDevice (devices[i]);
    setMobility(nodes.Get(i), Vector ((i/8)*20,(i%8)*20,0));
    
    sprintf(name, "%d", i);
    devices[i]->GetPhy ()->TraceConnect ("TrxState", std::string (name), MakeCallback (&StateChangeNotification));
  
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
    char name[5];
    devices[i+64] = CreateObject<LrWpanNetDevice> ();
    
    stringstream ss;
    ss << setbase(16) << setfill('0') << setw(2);
    ss << i+65;
    ss >> a;
    b = "00:"+a;
    devices[i+64]->SetAddress (Mac16Address (b.c_str()));
    devices[i+64]->SetChannel (channel);
    
    gw.Get(i)->AddDevice (devices[i+64]);
    NetDeviceContainer az = wifi.Install (wifiPhy, wifiMac, gw.Get(i));
    setMobility (gw.Get(i), Vector ((i/2)*80+30,(i%2)*80+30,0));
    devs.Add(az);
    
    sprintf(name, "%d", i+64);
    devices[i+64]->GetPhy ()->TraceConnect ("TrxState", std::string (name), MakeCallback (&StateChangeNotification));
    
    McpsDataConfirmCallback cb0;
    cb0 = MakeCallback (&DataConfirm);
    devices[i+64]->GetMac ()->SetMcpsDataConfirmCallback (cb0);
    
    McpsDataIndicationCallback cb1;
    cb1 = MakeCallback (&DataIndication);
    devices[i+64]->GetMac ()->SetMcpsDataIndicationCallback (cb1);
  }
  
  sink_dev = wifi.Install (wifiPhy, wifiMac, sink);
  setMobility (sink, Vector (200,200,0));
  wifiPhy.EnablePcapAll ("IoT-Test");  
  
  BasicEnergySourceHelper basicSourceHelper;
  // configure energy source
  basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (300));
  // install source
  sources = basicSourceHelper.Install (gw);
  // device energy model
  WifiRadioEnergyModelHelper radioEnergyHelper;
  // install device model
  DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (devs, sources);
	
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
iotTest::setMobility (Ptr<Node> node, Vector position)
{
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> pos;
  
  pos = CreateObject<ListPositionAllocator> ();
  pos->Add(position);
  
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (pos);
  mobility.Install (node);
}

void
iotTest::Run ()
{
  double avg_tx, avg_rx;
  double energyTx, energyRx, avgIdle, avgTx, avgRx;
  for (int i=0; i<64; i++) ScheduleNext (i);
	
  AnimationInterface anim ("iot-big.xml"); // Mandatory
  anim.UpdateNodeDescription (sink, "sink"); // Optional
  anim.UpdateNodeColor (sink, 0, 0, 255); // Optional
  anim.UpdateNodeSize ( sink->GetId(), 3.0, 3.0); // Optional
  for (uint32_t i = 0; i < 64; ++i)
    {
      anim.UpdateNodeDescription (nodes.Get (i), "node"); // Optional
      anim.UpdateNodeColor (nodes.Get (i), 255, 0, 0); // Optional
      anim.UpdateNodeSize ( nodes.Get(i)->GetId(), 1.0, 1.0); // Optional
    }
  for (uint32_t i = 0; i < 4; ++i)
    {
      anim.UpdateNodeDescription (gw.Get (i), "node"); // Optional
      anim.UpdateNodeColor (gw.Get (i), 0, 255, 0); // Optional
      anim.UpdateNodeSize ( gw.Get(i)->GetId(), 2.0, 2.0); // Optional
    }
	
  Simulator::Stop (Seconds (20.0));
  Simulator::Run ();
  
  avg_tx = avg_rx = 0.0;
  for(int i=0; i<68; i++) {
    avg_tx += tx_time[i].ToDouble(Time::US);
    avg_rx += rx_time[i].ToDouble(Time::US);
  }
  for (int i=0; i<64; i++) {
	  	total_gened += gened[i];
  }
	
  NS_LOG_UNCOND ("#pkts/#measurements/#drop/#gen: " << total_packet << " / " << measurementCount << " / " << m_dropped << " / " << total_gened);
  NS_LOG_UNCOND ("LrWpan Tx(ms)/Rx(ms): " << avg_tx/68 << " / " << std::fixed << avg_rx/68);

  energyTx = energyRx = avgIdle = avgTx = avgRx = 0.0;
  for (uint32_t i=0; i<4; i++) {
    Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources.Get (i));
    Ptr<DeviceEnergyModel> basicRadioModelPtr = basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
    Ptr<WifiRadioEnergyModel> ptr = DynamicCast<WifiRadioEnergyModel> (basicRadioModelPtr);
    NS_ASSERT (ptr != NULL);
    avgIdle += ptr->GetIdleTime().ToDouble(Time::MS);
    avgTx += ptr->GetTxTime().ToDouble(Time::MS);
    avgRx += ptr->GetRxTime().ToDouble(Time::MS);
    energyTx += ptr->GetTxTime().ToDouble(Time::MS) * ptr->GetTxCurrentA() * 3;
    energyRx += ptr->GetRxTime().ToDouble(Time::MS) * ptr->GetTxCurrentA() * 3;
  }
	
//  NS_LOG_UNCOND ("WiFi energy Cost");
  NS_LOG_UNCOND ("Avg Idle time(ms) / Avg Tx Time(ms) / Avg Rx Time(ms): " << avgIdle/4 << "/" << avgTx/4 << "/" << avgRx/4);
  NS_LOG_UNCOND ("Avg Tx energy(mJ) / Avg Rx energy(mJ): " << energyTx/4 << "/" << energyRx/4);
	
  Simulator::Destroy ();
  return ;
}

void
iotTest::TxPkt (int index)
{
  Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
  
  // Every sensor node transmit data to their gateway
	int i = (index/32)*2 + (index%8)/4;
    McpsDataRequestParams params;
    params.m_srcAddrMode = SHORT_ADDR;
    params.m_dstAddrMode = SHORT_ADDR;
    params.m_dstPanId = 0;
    params.m_dstAddr = devices[i+64]->GetMac ()->GetShortAddress ();
    params.m_msduHandle = 0;
    params.m_txOptions = TX_OPTION_ACK;

    Ptr<Packet> p0 = Create<Packet> (17);
    leach::LeachHeader hdr;
    Ptr<LrWpanNetDevice> dev1 = DynamicCast<LrWpanNetDevice> (nodes.Get (index)->GetDevice (0));
    hdr.SetDeadline (Now() + Seconds(5));
    p0->AddHeader (hdr);
    Simulator::Schedule (MilliSeconds (var->GetValue (0, 400)), &LrWpanMac::McpsDataRequest, dev1->GetMac (), params, p0);
    gened[index]++;
}

void
iotTest::ScheduleNext (int index)
{
  // Poisson or Periodic
  double nextTime;
  
  if (periodic) {
    nextTime = 1.0/m_lambda;
    Simulator::Schedule (Seconds(nextTime), &iotTest::ScheduleNext, this, index);
  } else {
    Ptr<UniformRandomVariable> m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();
    double p = m_uniformRandomVariable->GetValue (0,1);
    double poisson, expo = exp(-m_lambda);
    int k;
 
    poisson = expo;
    for(k=1; poisson < p; k++)
      {
        double temp = pow(m_lambda, k);
        for(int j=1; j<=k; j++) temp /= j;
        poisson += temp*expo;
      }
    k--;
    if(k>0) {
      nextTime = 1.0/k;
      Simulator::Schedule (Seconds(nextTime), &iotTest::ScheduleNext, this, index);
    } else {
      Simulator::Schedule (Seconds(1.0), &iotTest::ScheduleNext, this, index);
      return;
    }
  }
	
  TxPkt(index);
}

#ifdef DA_PROP
static bool
Proposal (int gw_index)
{
  // pick up those selected entry and send
  int expired = 0, expected;
  leach::LeachHeader hdr;
  Time deadLine = Now();
  deadLine += Seconds(gw_buffer[gw_index].size()/m_lambda);
  
  for (int i=0; i<(int)gw_buffer[gw_index].size(); i++)
    {
      Packet a (*gw_buffer[gw_index][i]);
      a.RemoveHeader(hdr);
  
//      NS_LOG_UNCOND("GetDeadline: " << hdr.GetDeadline() << ", UID: " << gw_buffer[gw_index][i]->GetUid() << ", Now: " << Now());
      if(hdr.GetDeadline() < Now())
        {
          // drop it
          NS_LOG_UNCOND("Drop");
          gw_buffer[gw_index].erase (gw_buffer[gw_index].begin()+i);
          m_dropped++;
          i--;
        }
    }
  
  for(uint32_t i=0; i<gw_buffer[gw_index].size(); i++) {
    Packet a (*gw_buffer[gw_index][i]);
    a.RemoveHeader(hdr);
	  
    if(hdr.GetDeadline() < deadLine) {
      expired++;
    }
  }
  expected = 16;
  
  if(expired >= expected || Now() > Seconds(48.5)) {
    return true;
  }
  return false;
}
#endif

#ifdef DA_OPT
static bool
OptTM (int gw_index)
{
  Time time = Now();
  uint32_t rewards[100], maxR = 0;
  uint32_t actions[100];
  static int step = 0;
  leach::LeachHeader hdr;
  
  for(int i=0; i<100; i++)
    {
      actions[i] = 0;
      rewards[i] = 0;
      for(uint j=0; j<gw_buffer[gw_index].size(); j++)
        {
          Packet a (*gw_buffer[gw_index][j]);
          a.RemoveHeader(hdr);

          if(hdr.GetDeadline() >= time) rewards[i] += hdr.GetDeadline().ToInteger(Time::MS) - time.ToInteger(Time::MS);
        }
      for(int j=1; j<i+step; j++)
        {
          rewards[i] += (j<8) ?30000-j*4000 :0;
        }
      time += Seconds(1/m_lambda);
    }
  
  for(int i=0; i<100; i++)
    {
      if(rewards[i] > maxR)
        {
          maxR = rewards[i];
        }
    }
  
  // wait=1, transmit=2
  for(int i=98; i>=0; i--)
    {
      if(rewards[i] < maxR)
        actions[i] = 1;
      else
        {
          double rb[100], rn[100];
          rb[99] = 1.0;
          rn[99] = 0.0;

          for(int k=98; k>i; k--)
            {
              rn[k] = max(0.0, i*rn[k+1]/(k+1) + rb[k+1]/(k+1));
              rb[k] = max(rewards[k], rn[k]);
            }

          if(rewards[i] >= (uint32_t)rb[i+1])
            actions[i] = 2;
          else
            actions[i] = 1;
        }
    }
  step++;
  
  if(actions[0] > 1 || Now() > Seconds(48.5))
    {
      return true;
    }
  
  return false;
}
#endif

#ifdef DA_CL
static bool
ControlLimit (int gw_index)
{
  static uint32_t threshold = (1/(log(1/0.1)*(log(1/0.1)+m_lambda)))+2;
  leach::LeachHeader hdr;
	
  for(int i=0; i<(int)gw_buffer[gw_index].size(); i++)
    {
      Packet a (*gw_buffer[gw_index][i]);
      a.RemoveHeader(hdr);

      if(hdr.GetDeadline() < Now())
        {
          gw_buffer[gw_index].erase (gw_buffer[gw_index].begin()+i);
          m_dropped++;
          i--;
        }
    }
    
  if(gw_buffer[gw_index].size() >= threshold || Now() > Seconds(48.5))
    {
      return true;
    }
  return false;
}
#endif

static bool
AggregationPolicy (int gw_index)
{
  // Implement data aggregation policy
  // and data addgregation function

  if (Simulator::Now() > Seconds(18.9)) return true;
#ifdef DA_PROP
  return Proposal(gw_index);
#endif
#ifdef DA_OPT
  return OptTM(gw_index);
#endif
#ifdef DA_CL
  return ControlLimit(gw_index);
#endif
  
  return true;
}

static void
Aggregate (Ptr<Packet> p, int gw_index)
{
  for (int i=0; i<(int)gw_buffer[gw_index].size(); i++)
    p->AddAtEnd (gw_buffer[gw_index][i]);
  gw_buffer[gw_index].clear ();
	
  return ;
}

static void
Enqueue (Ptr<Packet> p, int gw_index)
{
  gw_buffer[gw_index].push_back (p);
}

