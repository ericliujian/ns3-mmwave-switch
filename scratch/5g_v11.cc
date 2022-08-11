/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
*   Copyright (c) 2020 University of Padova, Dep. of Information Engineering,
*   SIGNET lab.
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation;
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include "ns3/mmwave-sidelink-spectrum-phy.h"
#include "ns3/mmwave-vehicular-net-device.h"
#include "ns3/mmwave-vehicular-helper.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/mobility-module.h"
#include "ns3/config.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/spectrum-helper.h"
#include "ns3/mmwave-spectrum-value-helper.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/core-module.h"

// need to include it for changing weather
#include "ns3/mmwave-vehicular-propagation-loss-model.h"


// include and usings for output results into csv file, config GUI invoked
#include <iostream>
#include <fstream>
#include <cstdlib> // for exit function
using std::ofstream;
using std::cerr;
using std::endl;

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
using namespace std;

using vec    = vector<double>;
using matrix = vector<vec   >;

#include "ns3/config-store-module.h"

NS_LOG_COMPONENT_DEFINE ("5G");

using namespace ns3;
using namespace millicar;


uint32_t g_rxPackets; // total number of received packets
uint32_t g_txPackets; // total number of transmitted packets

Time g_firstReceived; // timestamp of the first time a packet is received
Time g_lastReceived; // timestamp of the last received packet


double humidity;
double visibility;
double particleradius;
double txPower=15;
int iteration=0;

matrix readCSV( string filename )
{
   matrix M;

   ifstream in( filename );
   string line;
   while ( getline( in, line ) )                   // read a whole line of the file
   {
      stringstream ss( line );                     // put it in a stringstream (internal stream)
      vec row;
      string data;
      while ( getline( ss, data, ',' ) )           // read (string) items up to a comma
      {
         row.push_back( stod( data ) );            // use stod() to convert to double; put in row vector
      }
      if ( row.size() > 0 ) M.push_back( row );    // add non-empty rows to matrix
   }
   return M;
}


void MmWaveProp(double humidity,double visibility,double particleradius, double stepTime, int iteration){

matrix M = readCSV( "data/Blanding2021psaverage.csv" );

Ptr<MmWaveVehicularPropagationLossModel> propagationLossModel = CreateObject<MmWaveVehicularPropagationLossModel> ();

humidity = M[iteration][0];
visibility= M[iteration][3];
particleradius=M[iteration][2];

propagationLossModel->SetHumidity(humidity);
propagationLossModel->SetVisibility(visibility);
propagationLossModel->SetParticleRadius(particleradius);


std::cout << "Humidity:\t" << humidity << std::endl;
std::cout << "Visibility:\t" << visibility << std::endl;
std::cout << "Particle Radius:\t" << particleradius << std::endl;

iteration +=1;


/*
auto channel = DynamicCast<MmWaveVehicularNetDevice>(devs.Get(0))->GetPhy()->GetSpectrumPhy()->GetSpectrumChannel();
  PointerValue plm;
  channel->GetAttribute("PropagationLossModel", plm);
  Ptr<MmWaveVehicularPropagationLossModel> pathloss = DynamicCast<MmWaveVehicularPropagationLossModel>(plm.Get<PropagationLossModel>());

  pathloss->SetHumidity(humidity);

*/   


Simulator::Schedule(Seconds(stepTime), &MmWaveProp, humidity, visibility, particleradius, stepTime, iteration);

}



void computeRxPower(NetDeviceContainer devs, NetDeviceContainer devs2, NetDeviceContainer devs3) {
  //set up mmWave channel 28GHz
  auto channel = DynamicCast<MmWaveVehicularNetDevice>(devs.Get(0))->GetPhy()->GetSpectrumPhy()->GetSpectrumChannel();
  PointerValue plm;
  channel->GetAttribute("PropagationLossModel", plm);
  Ptr<MmWaveVehicularPropagationLossModel> pathloss = DynamicCast<MmWaveVehicularPropagationLossModel>(plm.Get<PropagationLossModel>());

  Ptr<MobilityModel> mobileNode0 = devs.Get(0)->GetNode()->GetObject<MobilityModel>();
  Ptr<MobilityModel> mobileNode1 = devs.Get(1)->GetNode()->GetObject<MobilityModel>();
  

  double RxPowerVal = pathloss ->DoCalcRxPower( txPower, mobileNode1, mobileNode0);
  double Weatherpathloss= pathloss ->Compute_Ad(28000000000);

  //double PL = pathloss -> GetLoss (mobileNode1, mobileNode0);
  


  std::cout << "\nThe value of the RxPOWER is: " << RxPowerVal << std::endl;
  std::cout << "\nThe value of the Weather pathloss is: " << Weatherpathloss << std::endl;
  
    
  // setup DSRC channel   5.9GHz
  auto channel2 = DynamicCast<MmWaveVehicularNetDevice>(devs2.Get(0))->GetPhy()->GetSpectrumPhy()->GetSpectrumChannel();
  PointerValue plm2;
  channel2->GetAttribute("PropagationLossModel", plm2);
  Ptr<MmWaveVehicularPropagationLossModel> pathloss2 = DynamicCast<MmWaveVehicularPropagationLossModel>(plm2.Get<PropagationLossModel>());
  pathloss2->SetFrequency (5900000000);

  Ptr<MobilityModel> mobileNode2 = devs2.Get(0)->GetNode()->GetObject<MobilityModel>();
  Ptr<MobilityModel> mobileNode3 = devs2.Get(1)->GetNode()->GetObject<MobilityModel>();
  

  double RxPowerVal2 = pathloss2 ->DoCalcRxPower( txPower, mobileNode2, mobileNode3);
  double Weatherpathloss2= pathloss2 ->Compute_Ad(5900000000);
    

  std::cout << "The value of the RxPOWER2 is: " << RxPowerVal2 << "\n"<<std::endl;
  std::cout << "\nThe value of the Weather pathloss2 is: " << Weatherpathloss2 << std::endl;


  // setup LTE channel  2.1GHz
  auto channel3 = DynamicCast<MmWaveVehicularNetDevice>(devs3.Get(0))->GetPhy()->GetSpectrumPhy()->GetSpectrumChannel();
  PointerValue plm3;
  channel3->GetAttribute("PropagationLossModel", plm3);
  Ptr<MmWaveVehicularPropagationLossModel> pathloss3 = DynamicCast<MmWaveVehicularPropagationLossModel>(plm3.Get<PropagationLossModel>());
  pathloss3->SetFrequency (2100000000);

  Ptr<MobilityModel> mobileNode4 = devs3.Get(0)->GetNode()->GetObject<MobilityModel>();
  Ptr<MobilityModel> mobileNode5 = devs3.Get(1)->GetNode()->GetObject<MobilityModel>();
  

  double RxPowerVal3 = pathloss3 ->DoCalcRxPower( txPower, mobileNode4, mobileNode5);
  double Weatherpathloss3= pathloss3 ->Compute_Ad(2100000000);
    

  std::cout << "The value of the RxPOWER3 is: " << RxPowerVal3 << "\n"<<std::endl;
  std::cout << "\nThe value of the Weather pathloss3 is: " << Weatherpathloss3 << std::endl;

  std::ofstream outdata; // outdata is like cin
  
  outdata.open("RxPower28preweatherconstantpsv0.005d5.csv", std::ofstream::app); // opens the file
   if( !outdata ) { // file couldn't be opened
      cerr << "Error: file could not be opened" << endl;
      exit(1);
   }
  outdata << RxPowerVal<<","<<RxPowerVal2<<","<<RxPowerVal3<<","<<Weatherpathloss<<","<<Weatherpathloss2<<","<<Weatherpathloss3<<endl;


 // outdata << PL<< endl;
  outdata.close();

 
  //Simulator::Schedule(Seconds(stepTime), &computeRxPower, devs, stepTime);
}
 
void
GetDistance_From (Ptr<Node> node1, Ptr<Node> node2)
{

  Ptr<MobilityModel> model1 = node1->GetObject<MobilityModel>();
  Ptr<MobilityModel> model2 = node2->GetObject<MobilityModel>();
  double distance = model1->GetDistanceFrom (model2);
  //return distance;
  
  std::cout << "Distance:\t" << distance << std::endl;
  
  std::ofstream outdata; // outdata is like cin
  outdata.open("distance.csv", std::ofstream::app); // opens the file
   if( !outdata ) { // file couldn't be opened
      cerr << "Error: file could not be opened" << endl;
      exit(1);
   }
  outdata << distance<<endl;


 // outdata << PL<< endl;
  outdata.close();
  
  //Simulator::Schedule(Seconds(stepTime), &MmWaveProp, humidity, visibility, particleradius, stepTime, iteration);
} 



static void Rx (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
 g_rxPackets++;
 
 SeqTsHeader header;

 p->PeekHeader(header);

 *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << p->GetSize() << "\t" << header.GetSeq() << "\t" << header.GetTs().GetSeconds() << "\t"<<g_rxPackets<<std::endl;

 if (g_rxPackets > 1)
 {

   g_lastReceived = Simulator::Now();
 }
 else
 {
   g_firstReceived = Simulator::Now();
 }
}



int main (int argc, char *argv[])
{
  // This script creates two nodes moving at 20 m/s, placed at a distance of intraGroupDistance m.
  // These nodes exchange packets through a UDP application,
  // and they communicate using a wireless channel.
  // This V1 use RxPower to check receiver's power in dBm



  // system parameters
  double bandwidth ; // bandwidth in Hz
  double frequency ; // the carrier frequency
  uint32_t numerology=3 ; // the numerology

  // applications
  uint32_t packetSize = 1024; // UDP packet size in bytes
  uint32_t startTime = 0.05; // application start time in milliseconds
  uint32_t endTime = 100; // application end time in milliseconds

  uint32_t timeRes = 1; // 
  
  uint32_t interPacketInterval; // interpacket interval in microseconds

  // mobility
  double speed; // speed of the vehicles m/s
  double intraGroupDistance ; // distance between two vehicles belonging to the same group

  //three weather parameters
  double particleradius;
  double visibility;
  double humidity;

  // channel and scenario choose
  std::string channel_condition;
  std::string scenario;

  double stepTime = 0.001;
  //double stepTime2= 0.001;

  CommandLine cmd;
  //
  cmd.AddValue ("bandwidth", "used bandwidth", bandwidth);
  cmd.AddValue ("frequency", "set the carrier frequency", frequency);
  cmd.AddValue ("numerology", "set the numerology to use at the physical layer", numerology);
  cmd.AddValue ("iip", "inter packet interval, in microseconds", interPacketInterval);
  
      
  // cmd new add values
  cmd.AddValue ("speed", "set vehicle speed", speed);
  cmd.AddValue ("intraGroupDistance", "distance between two vehicles belonging to the same group, y-coord", intraGroupDistance);
  cmd.AddValue("channel_condition", "Defines the channel condition", channel_condition);
  cmd.AddValue("scenario", "Defines the scenario where the communication takes place", scenario);

  //cmd add weather impacts
  cmd.AddValue("Pvalue", "Particle radius value", particleradius);
  cmd.AddValue("Vvalue", "Visibility value", visibility);
  cmd.AddValue("Hvalue", "H valueeee", humidity);

  cmd.Parse (argc, argv);

  Config::SetDefault ("ns3::MmWaveSidelinkMac::UseAmc", BooleanValue (true));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (frequency));
  
  // need to change different bandwidth for mmWave and LTE
  //Config::SetDefault ("ns3::MmWaveVehicularHelper::Bandwidth", DoubleValue (bandwidth));
  //Config::SetDefault ("ns3::MmWaveVehicularHelper::Numerology", UintegerValue (numerology));
  
  // set channelcondition and scenario
  Config::SetDefault ("ns3::MmWaveVehicularPropagationLossModel::ChannelCondition", StringValue (channel_condition));
  Config::SetDefault("ns3::MmWaveVehicularPropagationLossModel::Scenario", StringValue(scenario));

  // set weather impacts + frequency in MmWaveVehicularPropagationLossModel
  //Config::SetDefault("ns3::MmWaveVehicularPropagationLossModel::ParticleRadius", DoubleValue(particleradius));
  //Config::SetDefault("ns3::MmWaveVehicularPropagationLossModel::VVisibility", DoubleValue(visibility));
  //Config::SetDefault("ns3::MmWaveVehicularPropagationLossModel::HHumidity", DoubleValue(humidity));
  
  //Since we create two MmWaveVehicularPropagationLossModel and we need to turn off setdefault
  //Config::SetDefault("ns3::MmWaveVehicularPropagationLossModel::Frequency", DoubleValue(frequency));


  Config::SetDefault ("ns3::MmWaveVehicularPropagationLossModel::Shadowing", BooleanValue (false));
  Config::SetDefault ("ns3::MmWaveVehicularSpectrumPropagationLossModel::UpdatePeriod", TimeValue (MilliSeconds (1)));
  Config::SetDefault ("ns3::MmWaveVehicularAntennaArrayModel::AntennaElements", UintegerValue (16));
  Config::SetDefault ("ns3::MmWaveVehicularAntennaArrayModel::AntennaElementPattern", StringValue ("3GPP-V2V"));
  Config::SetDefault ("ns3::MmWaveVehicularAntennaArrayModel::IsotropicAntennaElements", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveVehicularAntennaArrayModel::NumSectors", UintegerValue (2));
  
  Config::SetDefault ("ns3::MmWaveVehicularNetDevice::RlcType", StringValue("LteRlcUm"));
  Config::SetDefault ("ns3::MmWaveVehicularHelper::SchedulingPatternOption", EnumValue(2)); // use 2 for SchedulingPatternOption=OPTIMIZED, 1 or SchedulingPatternOption=DEFAULT
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (500*1024));

 

  // create the nodes
  NodeContainer n;
  n.Create (2);
  
  // create the mobility models
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (n);

  n.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,0));
  n.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (0, speed, 0));

  n.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (0, intraGroupDistance,  0));
  n.Get (1)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (0, speed, 0));
  
  //double distance = (n.Get (0)->GetObject<MobilityModel> ())->GetDistanceFrom (n.Get (1)->GetObject<MobilityModel> ());

  // create and configure the helper
  Ptr<MmWaveVehicularHelper> helper = CreateObject<MmWaveVehicularHelper> ();
  helper->SetBandwidth(bandwidth);
  helper->SetNumerology (3);
  helper->SetPropagationLossModelType ("ns3::MmWaveVehicularPropagationLossModel");
  helper->SetSpectrumPropagationLossModelType ("ns3::MmWaveVehicularSpectrumPropagationLossModel");
  NetDeviceContainer devs = helper->InstallMmWaveVehicularNetDevices (n);
  
  
  Ptr<MmWaveVehicularHelper> helper2 = CreateObject<MmWaveVehicularHelper> ();
  helper2->SetBandwidth(2e7);
  helper2->SetNumerology (2);
  helper2->SetPropagationLossModelType ("ns3::MmWaveVehicularPropagationLossModel");
  helper2->SetSpectrumPropagationLossModelType ("ns3::MmWaveVehicularSpectrumPropagationLossModel");
  NetDeviceContainer devs2 = helper2->InstallMmWaveVehicularNetDevices (n);
  
  
  Ptr<MmWaveVehicularHelper> helper3 = CreateObject<MmWaveVehicularHelper> ();
  helper3->SetBandwidth(2e7);
  helper3->SetNumerology (2);
  helper3->SetPropagationLossModelType ("ns3::MmWaveVehicularPropagationLossModel");
  helper3->SetSpectrumPropagationLossModelType ("ns3::MmWaveVehicularSpectrumPropagationLossModel");
  NetDeviceContainer devs3 = helper3->InstallMmWaveVehicularNetDevices (n);
  
  

  // Install the TCP/IP stack in the two nodes
  InternetStackHelper internet;
  internet.Install (n);
  
   
  Ipv4AddressHelper ipv4;

  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devs);
  

  // Need to pair the devices in order to create a correspondence between transmitter and receiver
  // and to populate the < IP addr, RNTI > map.
  helper->PairDevices(devs);


  // Set the routing table
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting (n.Get (0)->GetObject<Ipv4> ());
  staticRouting->SetDefaultRoute (n.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal () , 2 );

  NS_LOG_DEBUG("IPv4 Address node 0: " << n.Get (0)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());
  NS_LOG_DEBUG("IPv4 Address node 1: " << n.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());

  Ptr<mmwave::MmWaveAmc> m_amc = CreateObject <mmwave::MmWaveAmc> (helper->GetConfigurationParameters());

  // setup the applications
  Config::SetDefault ("ns3::UdpClient::MaxPackets", UintegerValue (0xFFFFFFFF));
  Config::SetDefault ("ns3::UdpClient::Interval", TimeValue (MicroSeconds (interPacketInterval)));
  Config::SetDefault ("ns3::UdpClient::PacketSize", UintegerValue (packetSize));
  
    

  // create the applications
  uint32_t port = 4000;

  UdpEchoServerHelper server (port);
  
  ApplicationContainer echoApps = server.Install (n.Get (1));
  echoApps.Start (Seconds (0.0));


  AsciiTraceHelper asciiTraceHelper;
  Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream ("28Ghz.txt");
  echoApps.Get(0)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, stream));


  UdpClientHelper client (n.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port);
  ApplicationContainer apps = client.Install (n.Get (0));
  
  apps.Start (MilliSeconds (startTime));

  // set the application start/end time

  
  
  double throughput = (double(g_rxPackets)*(double(packetSize)*8)/double( g_lastReceived.GetSeconds() - g_firstReceived.GetSeconds()))/1e6;
  
 
 
  

  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("11.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i2 = ipv4.Assign (devs2);

  helper2->PairDevices(devs2);


  // create the applications
  uint32_t port2 = 4001;

  UdpEchoServerHelper server2 (port2);
  
  ApplicationContainer echoApps2 = server2.Install (n.Get (1));
  echoApps2.Start (Seconds (0.0));


  AsciiTraceHelper asciiTraceHelper2;
  Ptr<OutputStreamWrapper> stream2 = asciiTraceHelper2.CreateFileStream ("5.9Ghz.txt");
  echoApps2.Get(0)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, stream2));


  UdpClientHelper client2 (n.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port2);
  ApplicationContainer apps2 = client2.Install (n.Get (0));
  


  // set the application start/end time
  apps2.Start (MilliSeconds (startTime));


  double throughput2 = (double(g_rxPackets)*(double(packetSize)*8)/double( g_lastReceived.GetSeconds() - g_firstReceived.GetSeconds()))/1e6;
  
  
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("12.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i3 = ipv4.Assign (devs3);

  helper3->PairDevices(devs3);


  // create the applications
  uint32_t port3 = 4002;

  UdpEchoServerHelper server3 (port3);
  
  ApplicationContainer echoApps3 = server3.Install (n.Get (1));
  echoApps3.Start (Seconds (0.0));


  AsciiTraceHelper asciiTraceHelper3;
  Ptr<OutputStreamWrapper> stream3 = asciiTraceHelper3.CreateFileStream ("2.1Ghz.txt");
  echoApps3.Get(0)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, stream3));


  UdpClientHelper client3 (n.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port3);
  ApplicationContainer apps3 = client3.Install (n.Get (0));
  


  // set the application start/end time
  apps3.Start (MilliSeconds (startTime));


  double throughput3 = (double(g_rxPackets)*(double(packetSize)*8)/double( g_lastReceived.GetSeconds() - g_firstReceived.GetSeconds()))/1e6;
  
  // Invoke GUI just before entering Simulator::Run 
  /*GtkConfigStore config;
  config.ConfigureDefaults ();
  config.ConfigureAttributes ();*/
  
  matrix M = readCSV( "data/Blanding2021psaverage.csv");
  //write( M );

  std::cout << "----------- Humidity -----------" << std::endl;
  std::cout << "HUMIDITY+++++"<<M[0][0] << std::endl;
  
  humidity = M[0][0];
  visibility= M[0][3];
  particleradius= M[0][2];

  
  MmWaveProp(humidity, visibility, particleradius, stepTime, iteration);
  //MmWaveProp(devs,humidity, stepTime);

  for (size_t i = 0; i < endTime / timeRes; i++)
    {
      Simulator::Schedule (MilliSeconds(timeRes * (i+0.1)), &computeRxPower, devs, devs2,devs3);
      Simulator::Schedule (MilliSeconds(timeRes * (i+0.1)), &GetDistance_From, n.Get(0), n.Get(1));
    }

  //computeRxPower(devs, stepTime2);
  //GetDistance_From(n.Get (0), n.Get (1), stepTime);
  
  //Simulator::Schedule(MilliSeconds(0.001), &GetDistance_From, n.Get(0), n.Get(1));
  Simulator::Stop (MilliSeconds (endTime + 1));
   
    
  Simulator::Run ();
  Simulator::Destroy ();

 // double throughput = (double(g_rxPackets)*(double(packetSize)*8)/double( g_lastReceived.GetSeconds() - g_firstReceived.GetSeconds()))/1e6;
  
  std::cout << "----------- Statistics -----------" << std::endl;
  std::cout << "Packets size:\t\t" << packetSize << " Bytes" << std::endl;
  //std::cout << distance << std::endl;
  std::cout << "Packets received:\t" << g_rxPackets << std::endl;
  std::cout << "Average Throughput:\t" << throughput << " Mbps" << std::endl;
  std::cout << "Average Throughput2:\t" << throughput2 << " Mbps" << std::endl;
  std::cout << "First Received Time:\t" << g_firstReceived.GetSeconds() << std::endl;
  std::cout << "Last Received Time:\t" << g_lastReceived.GetSeconds() << std::endl;

  // create a result.csv file under ns3 default folder and output received packets, throughput, particle radius,visibility,humidity, frequency,
  // first and last packet received time, intraGroupDistance into the file.
/*  
  std::ofstream outdata; // outdata is like cin
  
  outdata.open("result5g_v1.csv", std::ofstream::app); // opens the file
   if( !outdata ) { // file couldn't be opened
      cerr << "Error: file could not be opened" << endl;
      exit(1);
   }
  outdata << g_rxPackets<<","<<throughput<<","<<RxPower<<","<<particleradius<<","<<visibility<<","<<humidity<<","<<frequency<<","<<g_firstReceived.GetSeconds()<<","<<g_lastReceived.GetSeconds()<<","<<intraGroupDistance << endl;
  outdata.close();
  */

  return 0;
}
