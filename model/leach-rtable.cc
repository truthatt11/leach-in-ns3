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
#include "leach-rtable.h"
#include "ns3/simulator.h"
#include <iomanip>
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LeachRoutingTable");
  
namespace leach {
RoutingTableEntry::RoutingTableEntry (Ptr<NetDevice> dev,
                                      Ipv4Address dst,
                                      Ipv4InterfaceAddress iface,
                                      Ipv4Address nextHop)
  : m_iface (iface),
    m_flag (VALID)
{
  m_ipv4Route = Create<Ipv4Route> ();
  m_ipv4Route->SetDestination (dst);
  m_ipv4Route->SetGateway (nextHop);
  m_ipv4Route->SetSource (m_iface.GetLocal ());
  m_ipv4Route->SetOutputDevice (dev);
}
RoutingTableEntry::~RoutingTableEntry ()
{
}
RoutingTable::RoutingTable ()
{
}

bool
RoutingTable::LookupRoute (Ipv4Address id,
                           RoutingTableEntry & rt)
{
  if (m_ipv4AddressEntry.empty ())
    {
//      NS_LOG_UNCOND("1");
      return false;
    }
  std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = m_ipv4AddressEntry.find (id);
  if (i == m_ipv4AddressEntry.end ())
    {
//      NS_LOG_UNCOND("2");
      return false;
    }
  rt = i->second;
  return true;
}

bool
RoutingTable::LookupRoute (Ipv4Address id,
                           RoutingTableEntry & rt,
                           bool forRouteInput)
{
  if (m_ipv4AddressEntry.empty ())
    {
      return false;
    }
  std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = m_ipv4AddressEntry.find (id);
  if (i == m_ipv4AddressEntry.end ())
    {
      return false;
    }
  if (forRouteInput == true && id == i->second.GetInterface ().GetBroadcast ())
    {
      return false;
    }
  rt = i->second;
  return true;
}

bool
RoutingTable::DeleteRoute (Ipv4Address dst)
{
  if (m_ipv4AddressEntry.erase (dst) != 0)
    {
      // NS_LOG_DEBUG("Route erased");
      return true;
    }
  return false;
}

uint32_t
RoutingTable::RoutingTableSize ()
{
  return m_ipv4AddressEntry.size ();
}

bool
RoutingTable::AddRoute (RoutingTableEntry & rt)
{
  std::pair<std::map<Ipv4Address, RoutingTableEntry>::iterator, bool> result = m_ipv4AddressEntry.insert (std::make_pair (
                                                                                                            rt.GetDestination (),rt));
  return result.second;
}

bool
RoutingTable::Update (RoutingTableEntry & rt)
{
  std::map<Ipv4Address, RoutingTableEntry>::iterator i = m_ipv4AddressEntry.find (rt.GetDestination ());
  if (i == m_ipv4AddressEntry.end ())
    {
      return false;
    }
  i->second = rt;
  return true;
}

void
RoutingTable::DeleteAllRoutesFromInterface (Ipv4InterfaceAddress iface)
{
  if (m_ipv4AddressEntry.empty ())
    {
      return;
    }
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i = m_ipv4AddressEntry.begin (); i != m_ipv4AddressEntry.end (); )
    {
      if (i->second.GetInterface () == iface)
        {
          std::map<Ipv4Address, RoutingTableEntry>::iterator tmp = i;
          ++i;
          m_ipv4AddressEntry.erase (tmp);
        }
      else
        {
          ++i;
        }
    }
}

void
RoutingTable::GetListOfAllRoutes (std::map<Ipv4Address, RoutingTableEntry> & allRoutes)
{
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i = m_ipv4AddressEntry.begin (); i != m_ipv4AddressEntry.end (); ++i)
    {
      if (i->second.GetDestination () != Ipv4Address ("127.0.0.1") && i->second.GetFlag () == VALID)
        {
          allRoutes.insert (
            std::make_pair (i->first,i->second));
        }
    }
}

void
RoutingTable::GetListOfDestinationWithNextHop (Ipv4Address nextHop,
                                               std::map<Ipv4Address, RoutingTableEntry> & unreachable)
{
  unreachable.clear ();
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = m_ipv4AddressEntry.begin (); i
       != m_ipv4AddressEntry.end (); ++i)
    {
      if (i->second.GetNextHop () == nextHop)
        {
          unreachable.insert (std::make_pair (i->first,i->second));
        }
    }
}

void
RoutingTableEntry::Print (Ptr<OutputStreamWrapper> stream) const
{
  *stream->GetStream () << std::setiosflags (std::ios::fixed) << m_ipv4Route->GetDestination () << "\t\t" << m_ipv4Route->GetGateway () << "\t\t"
                        << m_iface.GetLocal () << "\n";
}

void
RoutingTable::Print (Ptr<OutputStreamWrapper> stream) const
{
  
  *stream->GetStream () << "\nLEACH Routing table\n" << "DST\t\t\tDestination\t\tGateway\t\t\tInterface\n";
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator i = m_ipv4AddressEntry.begin (); i
       != m_ipv4AddressEntry.end (); ++i)
    {
      i->first.Print (*stream->GetStream());
      *stream->GetStream() << "\t\t";
      i->second.Print (stream);
    }
  *stream->GetStream () << "\n";
}

bool
RoutingTable::AddIpv4Event (Ipv4Address address,
                            EventId id)
{
  std::pair<std::map<Ipv4Address, EventId>::iterator, bool> result = m_ipv4Events.insert (std::make_pair (address,id));
  return result.second;
}

bool
RoutingTable::AnyRunningEvent (Ipv4Address address)
{
  EventId event;
  std::map<Ipv4Address, EventId>::const_iterator i = m_ipv4Events.find (address);
  if (m_ipv4Events.empty ())
    {
      return false;
    }
  if (i == m_ipv4Events.end ())
    {
      return false;
    }
  event = i->second;
  if (event.IsRunning ())
    {
      return true;
    }
  else
    {
      return false;
    }
}

bool
RoutingTable::ForceDeleteIpv4Event (Ipv4Address address)
{
  EventId event;
  std::map<Ipv4Address, EventId>::const_iterator i = m_ipv4Events.find (address);
  if (m_ipv4Events.empty () || i == m_ipv4Events.end ())
    {
      return false;
    }
  event = i->second;
  Simulator::Cancel (event);
  m_ipv4Events.erase (address);
  return true;
}

bool
RoutingTable::DeleteIpv4Event (Ipv4Address address)
{
  EventId event;
  std::map<Ipv4Address, EventId>::const_iterator i = m_ipv4Events.find (address);
  if (m_ipv4Events.empty () || i == m_ipv4Events.end ())
    {
      return false;
    }
  event = i->second;
  if (event.IsRunning ())
    {
      return false;
    }
  if (event.IsExpired ())
    {
      event.Cancel ();
      m_ipv4Events.erase (address);
      return true;
    }
  else
    {
      m_ipv4Events.erase (address);
      return true;
    }
}

EventId
RoutingTable::GetEventId (Ipv4Address address)
{
  std::map <Ipv4Address, EventId>::const_iterator i = m_ipv4Events.find (address);
  if (m_ipv4Events.empty () || i == m_ipv4Events.end ())
    {
      return EventId ();
    }
  else
    {
      return i->second;
    }
}
}
}