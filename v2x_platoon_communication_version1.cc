/* 
   第一个仿真的具体设计：
   共有2个platoon，每个platoon车的数量均为10，有1个基站 (目前只考虑这三种设备，其他设备不要考虑)。
   考虑platoon之间通过车头通信，platoon内部进行广播通信（车头对其他），platoon内部其他对车头目前采用单播--->可以暂不考虑这一块。
   只有车头可以和基站通信。
   Channel and NetDevice  ——> LTE-C-V2X 相关 
   Mobility:首先实现最简单的静态Mobility
   车车通过sidelink通信,利用好三种信道

   代码目前的问题：
   (1)sidelink部分->原理还不够清楚
   已经解决：
   (1)框架搭建  (2)常规模块代码的完全理解  （3）很多模块的基于platoon的修改
   SPS C-V2X！    
   调试LTE UE与eNodeB
*/
#include "ns3/lte-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/lte-v2x-helper.h"
#include "ns3/config-store.h"
#include "ns3/lte-hex-grid-enb-topology-helper.h"
#include <ns3/buildings-helper.h>
#include <ns3/cni-urbanmicrocell-propagation-loss-model.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/spectrum-analyzer-helper.h>
#include <ns3/multi-model-spectrum-channel.h>
#include "ns3/ns2-mobility-helper.h"
#include <cfloat>
#include <sstream>
using namespace ns3;

//Output
std::string simtime = "log_simtime_v2x.csv";
std::string rx_data = "log_rx_data_v2x.csv";
std::string tx_data = "log_tx_data_v2x.csv";
std::string connections = "log_connections_v2x.csv";
std::string positions = "log_position_v2x.csv";

Ptr<OutputStreamWrapper>log_connections;
Ptr<OutputStreamWrapper>log_simtime;
Ptr<OutputStreamWrapper>log_positions;
Ptr<OutputStreamWrapper>log_rx_data;
Ptr<OutputStreamWrapper>log_tx_data;


int ctr_totRx = 0 ;//receive packets number
int ctr_totTx = 0;//send packets number
double baseline = 150.0; //baseline distance 
int lenCam;

NodeContainer ueAllNodes;

void 
SidelinkV2xAnnouncementMacTrace(Ptr<Socket>socket)
{
    Ptr <Node> node = socket->GetNode(); 
    int id = node->GetId();
    int simTime = Simulator::Now().GetMilliSeconds(); 
    Ptr<MobilityModel> posMobility = node->GetObject<MobilityModel>();
    Vector posTx = posMobility->GetPosition();
   
   //check for each UE distance to transimitter
   for(uint32_t i=0;i<ueAllNodes.GetN();i++)
   {
      Ptr<MobilityModel>mob = ueAllNodes.Get(i)->GetObject<MobilityModel>();
      Vector posRx = mob->GetPosition();

      double distance = sqrt(pow((posTx.x-posRx.x),2.0)+pow((posTx.y-posRx.y),2.0));
      if(distance > 0 && distance < baseline)
      {
         ctr_totTx++;
      }
   }

   //Generate CAM
   std::ostringstream msgCam;
   msgCam<<id-1<<";"<<simTime<<";"<<(int)posTx.x<<";"<<(int)posTx.y<<'\0';
   Ptr<Packet>packet = Create<Packet>((uint8_t*)msgCam.str().c_str(),lenCam);//packet类型
   socket->Send(packet);//发送packet
   *log_tx_data->GetStream()<<ctr_totTx<<";"<<simTime<<";"<<id-1<<";"<<(int)posTx.y<<std::endl; //记录
}

void 
ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Node>node = socket->GetNode();
    Ptr<MobilityModel>posMobility = node->GetObject<MobilityModel>();
    Vector posRx = posMobility->GetPosition();
    Ptr<Packet>packet = socket->Recv();
    uint8_t *buffer = new uint8_t[packet->GetSize()];
    packet->CopyData(buffer,packet->GetSize());
    std::string s = std::string((char*)buffer);

    size_t pos=0;
    std::string copy = s;
    std::string token;
    int posTx_x;
    int posTx_y;
    for(int i=0;i<3;i++) //区分出；
    {
      if (copy.find(";") != std::string::npos)
        {
            pos = copy.find(";");
            token = copy.substr(0,pos);
            if(i == 2)
            {
                posTx_x = atoi(token.c_str());
            }
            copy.erase (0,pos+1);
        }  
    } 
    posTx_y = atoi(copy.c_str()); 
    double distance = sqrt(pow((posTx_x - posRx.x),2.0)+pow((posTx_y - posRx.y),2.0));
    if (distance <= baseline)
    {         
        int id = node->GetId();
        int simTime = Simulator::Now().GetMilliSeconds();
        ctr_totRx++; 
        *log_rx_data->GetStream() << ctr_totRx << ";" << simTime << ";"  << id-1 << ";" << s << std::endl; 
    }
}

void PrintStatus(uint32_t s_period, Ptr<OutputStreamWrapper>log_simtime)
{
   if(ctr_totRx > ctr_totTx)
    {
       ctr_totRx = ctr_totTx;
    }
    *log_simtime->GetStream() << Simulator::Now ().GetSeconds () << ";" << ctr_totRx << ";" << ctr_totTx << ";" << (double) ctr_totRx / ctr_totTx << std::endl; 
    std::cout << "t=" <<  Simulator::Now().GetSeconds() << "\t Rx/Tx="<< ctr_totRx << "/" << ctr_totTx << "\t PRR=" << (double) ctr_totRx / ctr_totTx << std::endl;
    Simulator::Schedule(Seconds(s_period), &PrintStatus, s_period,log_simtime);
}

int main(int argc,char *argv[])
{
   //CommandLine Argument
   CommandLine cmd;
   cmd.Parse(argc,argv);

   //generate csv file to record simulation
   AsciiTraceHelper ascii;
   log_simtime = ascii.CreateFileStream(simtime);
   log_rx_data = ascii.CreateFileStream(rx_data);
   log_tx_data = ascii.CreateFileStream(tx_data);
   log_connections = ascii.CreateFileStream(connections);
   log_positions = ascii.CreateFileStream(positions);

   int simTime = 100;  //simulation time in seconds
   int VehiclePerPlatoon = 10;   //vehicles of each platoon
   int platoonNum  = 2;   //number of platoons
   int VehicleNum;  // VehicleNum = platoonNum*platoonLen 
   double vehicleInterD = 1.0; //distance between each vehicle in a platoon 
  // int preD = 0;   //
   double platoonInterD = 10; //distance between two platoons
   double ueTxPower = 23.0; //ue tx power in dBm
   VehicleNum = platoonNum*VehiclePerPlatoon;
   lenCam = 190; //cooperative awareness message length   50bytes--300bytes

  //Set Configuration in PHY Layer
  //Set the UEs power in dBm---->recognize Vehicle to UE
  Config::SetDefault ("ns3::LteUePhy::TxPower",DoubleValue(ueTxPower)); 
  Config::SetDefault("ns3::LteUePhy::RsrpUeMeasThreshold",DoubleValue(-10.0));

  //Enable V2X Communication on PHY Layer
  Config::SetDefault("ns3::LteUePhy::EnableV2x",BooleanValue(true));

  // Set power->for different channels(specific usage of these channels?)
  Config::SetDefault ("ns3::LteUePowerControl::Pcmax", DoubleValue (ueTxPower));
  Config::SetDefault ("ns3::LteUePowerControl::PsschTxPower", DoubleValue (ueTxPower));
  Config::SetDefault ("ns3::LteUePowerControl::PscchTxPower", DoubleValue (ueTxPower));

  //Set COnfiguration in MAC Layer
  //Configure for UE selected
  uint16_t slBandwidth ; // sidelink bandwidth
  uint32_t mcs = 20; //modulation and coding scheme
  uint16_t pRsvp = 100; //Resource reservation interval 
  double probResourceKeep = 0.0;    // Probability to select the previous resource again
  uint16_t t1 = 4; uint16_t t2 = 100;   //T1,T2 value of selection window
  uint16_t sizeSubchannel = 10;           // Number of RBs per subchannel
  uint16_t numSubchannel = 3;             // Number of subchannels per subframe
  uint16_t startRbSubchannel = 0;         // Index of first RB corresponding to subchannelization
  slBandwidth = sizeSubchannel*numSubchannel;  //需要结合adjacencyPsschPscch修改
  Config::SetDefault ("ns3::LteUeMac::UlBandwidth",UintegerValue(slBandwidth)); //sidelink bandwith 
  //omit V2xHarq
  //omit adjacencyPscchPssch
  bool adjacencyPscchPssch = true; //相邻与否决定了subchannel scheme 从而决定了slBandwidth
  //omit partialsensing
  Config::SetDefault("ns3::LteUeMac::SlGrantMcs",UintegerValue(mcs));
  Config::SetDefault("ns3::LteUeMac::SlPrsvp",UintegerValue(pRsvp));
  Config::SetDefault ("ns3::LteUeMac::SlProbResourceKeep", DoubleValue(probResourceKeep));
  //Config::SetDefault ("ns3::LteUeMac::SlProbResourceKeep", DoubleValue(probResourceKeep));
  Config::SetDefault ("ns3::LteUeMac::SelectionWindowT1", UintegerValue(t1));
  Config::SetDefault ("ns3::LteUeMac::SelectionWindowT2", UintegerValue(t2));
  // problem in config
  
  ueAllNodes.Create(VehicleNum); //将每个Vehicle添加到Node Container 中,每个Vehicle看作一个ueNode
  
  //静态mobility配置
  MobilityHelper mobVeh;
  mobVeh.SetMobilityModel("ns3::ConstantPositionMobilityModel");//Constant Position
  Ptr<ListPositionAllocator>staticVeh[ueAllNodes.GetN()];//allocate initial node positions
  //0-9号车为Platoon1  10-19号车为Platoon2  Platoon 1&2之间间隔为platoonInterD
  //考虑在y上变化,x=0,z=0  
  int position_now = 0;
  for (uint32_t i=0;i<ueAllNodes.GetN();i++)
  {
   if(i==10)//是下一个platoon
   {
      staticVeh[i] = CreateObject<ListPositionAllocator>();
      staticVeh[i]->Add(Vector(0,position_now,0));
      position_now = position_now + platoonInterD; 
      mobVeh.SetPositionAllocator(staticVeh[i]);
      mobVeh.Install(ueAllNodes.Get(i));
      continue;
   }
   staticVeh[i] = CreateObject<ListPositionAllocator>(); 
   staticVeh[i]->Add(Vector(0,position_now,0));
   position_now = position_now + vehicleInterD;
   mobVeh.SetPositionAllocator(staticVeh[i]);
   mobVeh.Install(ueAllNodes.Get(i));
  }
  
  //EPC & LTE 无特别之处，在lte example 中的lena-simple-epc.cc可找到类似写法 ，常规
  //EPC(Evovled Packet Core)配置->Related  to the structure of EPC
  Ptr<PointToPointEpcHelper>epcHelper = CreateObject<PointToPointEpcHelper>();
  Ptr<Node>pgw = epcHelper->GetPgwNode(); // connect the PGW to the internet 

  //LTE Helper
  Ptr<LteHelper>lteHelper = CreateObject<LteHelper>();
  lteHelper->SetEpcHelper(epcHelper);
  lteHelper->DisableNewEnbPhy(); //disable eNBs for out-of-coverage modelling

  //V2X--->如果不是V2X的话，这里直接PointtoPointHepler连接UE & eNodeB即可
  Ptr<LteV2xHelper>lteV2xHelper = CreateObject<LteV2xHelper>();
  lteV2xHelper->SetLteHelper(lteHelper);

  //Configure eNB's antenna parameters before deploying them
  lteHelper->SetEnbAntennaModelType("ns3::NistParabolic3dAntennaModel");

  //Set Pathloss Model
  lteHelper->SetAttribute("UseSameUlDlPropagationCondition",BooleanValue(true)); // ? UseSameUlDlPropagationCondition?
  //Config::SetDefault("ns3::CniUrbanmicrocellPropagationLossModel",DoubleValue(5800e6));
  lteHelper->SetAttribute("PathlossModel",StringValue("ns3::CniUrbanmicrocellPropagationLossModel"));

  //Create eNB Container
  NodeContainer eNodeB;
  eNodeB.Create(1); // 1个基站

  //Topology eNodeB
  Ptr<ListPositionAllocator>pos_eNB = CreateObject<ListPositionAllocator>();
  pos_eNB ->Add(Vector(0,-10,0));

  //Install mobility eNodeB
  MobilityHelper mob_eNB;
  mob_eNB.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mob_eNB.SetPositionAllocator(pos_eNB);
  mob_eNB.Install(eNodeB);

  //Install Service
  NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(eNodeB);

  //LTE user documentation about "mobility"
  BuildingsHelper::Install(eNodeB);
  BuildingsHelper::Install(ueAllNodes);
  BuildingsHelper::MakeMobilityModelConsistent();

  //install LTE devices to all UEs
  lteHelper->SetAttribute("UseSidelink",BooleanValue(true));
  NetDeviceContainer ueDevs = lteHelper->InstallUeDevice(ueAllNodes);
  
  //install IP stack on the UEs
  InternetStackHelper internet;
  internet.Install(ueAllNodes);
                                                                                                                                                                                                         
  //Assign IP address to UEs
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address(ueDevs);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  //question in routing (need to redesign)
  for(uint32_t i=0; i<ueAllNodes.GetN(); i++)
  {
     Ptr <Node> ueNode = ueAllNodes.Get(i);
     Ptr<Ipv4StaticRouting>ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
     ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(),1);
  } 

  //attach UE to best available eNB --->Attach platoon header to eNB，other vehicles do not attach to eNB
  lteHelper->Attach(ueDevs.Get(0));
  lteHelper->Attach(ueDevs.Get(10)); 

  //create sidelink groups
  std::vector<NetDeviceContainer>txGroups;
  txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueDevs,VehicleNum); // (?) sidelink group ？？ associate UEs for vehicular broadcast communication

  //in platoon, only the 2 Platoon header can broadcast within the platoon
  txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueDevs,1); //这里，在版本1中，仅Platoon1的header可以广播
  //需要大改---->从node container开始改，每一个platoon的vehicle形成一个nodecontainer 同时对应一个NetDeviceContainer
    
  
  lteV2xHelper->PrintGroups(txGroups);

  //compute average number to receivers associated per transmitter and vice versa
  double totalRxs = 0;
  std::map<int,int>txPerUeMap;
  std::map<int,int>groupsPerUe; 
  //注意STL的使用

  std::vector<NetDeviceContainer>::iterator glt; // 定义一个迭代器来遍历元素
  for(glt=txGroups.begin();glt!=txGroups.end();glt++)
  {
      int numDevs = glt->GetN();
      totalRxs += numDevs - 1;
      int nld;

      for(int i=1;i<numDevs;i++)
      {
         nld=glt->Get(i)->GetNode()->GetId(); //不是很明白
         txPerUeMap[nld]++; //txPerUeMap(number of receivers associated per transimitter)
      }
  }

  double totalTxPerUe = 0;
  std::map<int,int>::iterator mlt;
  for(mlt=txPerUeMap.begin();mlt!=txPerUeMap.end();mlt++)
  {
      totalTxPerUe += mlt->second;
      groupsPerUe[mlt->second]++; //groupsPerUe(number of transimitter associated per receiver)
  }

  //install applications

  //Application setup
  std::vector<int>groupL2Addresses;
  int groupL2Address = 0x00;
  Ipv4AddressGenerator::Init(Ipv4Address("225.0.0.0"),Ipv4Mask("255.0.0.0")); // ipaddress = 225.0.0.0
  //Ipv4Address clientRespondersAddress = Ipv4AddressGenerator::NextAddress(Ipv4Mask("255.0.0.0"));
  Ipv4Address clientAddress = Ipv4AddressGenerator::NextAddress(Ipv4Mask("255.0.0.0"));

   int application_port = 8000; //application port
   NetDeviceContainer activeTxUes;

   //set sidelink v2x traces--->Have question!
   for(glt=txGroups.begin();glt!=txGroups.end();glt++)  //定义的每个sidelink组
   {
      //Create Sidelink bearers(carriers)
      //Use Tx for the group transmitter and Rx for all recerivers
      //Split Tx/Rx

      NetDeviceContainer txUe((*glt).Get(0)); // *glt指向的NetDevices中的第一个用于发送UE  即group中的tx
      activeTxUes.Add(txUe);
      NetDeviceContainer rxUes = lteV2xHelper->RemoveNetDevice((*glt),txUe.Get(0));//其他全部用于接收UE->广播 （为rx）
      Ptr<LteSlTft>tft = Create<LteSlTft>(LteSlTft::TRANSMIT,clientAddress,groupL2Address); 
      lteV2xHelper->ActivateSidelinkBearer(Seconds(0.0),txUe,tft);
      tft = Create<LteSlTft>(LteSlTft::RECEIVE,clientAddress,groupL2Address);
      lteV2xHelper->ActivateSidelinkBearer(Seconds(0.0),rxUes,tft);
      /*
        用户的IP数据包需要映射到不同的EPS Bearer，以获得相应的QoS保障。这样的映射关系是通过TFT（Traffic Flow Template）和其中的Packet Filters来实现的。
        TFT是映射到相应EPS Bearer的所有PacketFilter 的集合，Packet Filter表示将用户的一种业务数据流（SDF，Service DataFlow）映射到相应的EPS Bearer上，Packet Filter通常包括源/目的IP 地址，源/目的IP端口号，协议号等内容。
        专有的EPS Bearer必须有与之相应的TFT。相反的，缺省的EPS Bear通常并不配置特定的TFT，或者说，配置的是通配TFT，这样所有不能映射到专有EPS Bearer的IP数据包会被映射到缺省的EPS Bearer上。
        在专有的EPS Bearer被释放的情况下，原来映射到专有EPS Bearer上的数据包也会被重新路由到相应的缺省EPS Bearer上。TFT分为上行和下行两个方向，其中，上行的TFT在UE侧对上行的数据包进行过滤和映射。下行的TFT在PDN侧对下行的数据包进行过滤和映射。
      */

      //Individual Socket Traffic Broadcast everyone
      Ptr<Socket> host = Socket::CreateSocket(txUe.Get(0)->GetNode(),TypeId::LookupByName("ns3::UdpSocketFactory"));
      host->Bind();
      host->Connect(InetSocketAddress(clientAddress,application_port));
      host->SetAllowBroadcast(true);
      host->ShutdownRecv();
      /*
        Socket是应用层与TCP/IP协议族通信的中间软件抽象层，它是一组接口。
        在设计模式中，Socket其实就是一个门面模式，它把复杂的TCP/IP协议族隐藏在Socket接口后面，
        对用户来说，一组简单的接口就是全部，让Socket去组织数据，以符合指定的协议。
        https://blog.csdn.net/pashanhu6402/article/details/96428887
      */

      Ptr<LteUeMac>ueMac = DynamicCast<LteUeMac>(txUe.Get(0)->GetObject<LteUeNetDevice>()->GetMac());
      ueMac->TraceConnectWithoutContext("SidelinkV2xAnnouncement", MakeBoundCallback(&SidelinkV2xAnnouncementMacTrace,host));

      Ptr<Socket>sink = Socket::CreateSocket(txUe.Get(0)->GetNode(),TypeId::LookupByName("ns3::UdpSocketFactory"));
      sink->Bind(InetSocketAddress(Ipv4Address::GetAny(),application_port));
      sink->SetRecvCallback(MakeCallback(&ReceivePacket));
      //host and sink
      //host ---> tx   sink ---> rx

      //store and increment address
      groupL2Addresses.push_back(groupL2Address);
      groupL2Address++;
      clientAddress = Ipv4AddressGenerator::NextAddress(Ipv4Mask("255.0.0.0"));
   }

   //sidelink configuration
   Ptr<LteUeRrcSl>ueSidelinkConfiguration = CreateObject<LteUeRrcSl>();
   ueSidelinkConfiguration->SetSlEnabled(true);
   ueSidelinkConfiguration->SetV2xEnabled(true);

   LteRrcSap::SlV2xPreconfiguration preconfiguration;
   preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.carrierFreq = 54890;
   preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.slBandwidth = slBandwidth;

   preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.nbPools = 1;
   preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.nbPools = 1;

   SlV2xPreconfigPoolFactory pFactory;
   pFactory.SetHaveUeSelectedResourceConfig(true);
   pFactory.SetSlSubframe(std::bitset<20>(0xFFFFF));
   pFactory.SetAdjacencyPscchPssch(adjacencyPscchPssch);
   pFactory.SetSizeSubchannel(sizeSubchannel);
   pFactory.SetNumSubchannel(numSubchannel);
   pFactory.SetStartRbSubchannel(startRbSubchannel);
   pFactory.SetStartRbPscchPool(0);
   pFactory.SetDataTxP0(-4);
   pFactory.SetDataTxAlpha(0.9);

   preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0] = pFactory.CreatePool();
   preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0] = pFactory.CreatePool();
   ueSidelinkConfiguration->SetSlV2xPreconfiguration(preconfiguration);

   //Print Configuration
   *log_rx_data->GetStream()<<"RxPackets;RxTime;RxId;TxId;TxTime;xPos;yPos"<<std::endl;
   *log_tx_data->GetStream()<<"TxPackets;TxTime;TxId;xPos;yPos"<<std::endl;

   lteHelper->InstallSidelinkV2xConfiguration(ueDevs,ueSidelinkConfiguration);

   lteHelper->EnableTraces();

   *log_simtime->GetStream()<<"Simtime;TotalRx;TotalTx;PRR"<<std::endl;

   Simulator::Schedule(Seconds(1),&PrintStatus,1,log_simtime);

   Simulator::Stop(MilliSeconds(simTime*1000+40));
   Simulator::Run();
   Simulator::Destroy();
   
   return 0;


  // //Platoon场景下VANETs架构实现（应用层&EDCA机制）

  // //create platoon nodes
  // NodeContainer platoonNodes;
  // platoonNodes.Create(nodeNum);
  
  // //LTE-C-V2X
  // // EPC
  // Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
  // Ptr<Node> pgw = epcHelper->GetPgwNode();

  // // LTE Helper
  // Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
  // lteHelper->SetEpcHelper(epcHelper);
  // lteHelper->DisableNewEnbPhy(); // Disable eNBs for out-of-coverage modelling
    
  // // V2X 
  // Ptr<LteV2xHelper> lteV2xHelper = CreateObject<LteV2xHelper> ();
  // lteV2xHelper->SetLteHelper (lteHelper); 

  // // Channel: Set pathloss model 
  // // FIXME: InstallEnbDevice overrides PathlossModel Frequency with values from Earfcn
  // // 
  // lteHelper->SetAttribute ("UseSameUlDlPropagationCondition", BooleanValue(true));
  // Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", StringValue ("54990"));
  // //Config::SetDefault ("ns3::CniUrbanmicrocellPropagationLossModel::Frequency", DoubleValue(5800e6));
  // lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::CniUrbanmicrocellPropagationLossModel"));

  // //Create eNodeB
  // int numeNodeB = 1;
  // NodeContainer.Create(numeNodeB); // 先不考虑是否需要加上eNodeB的问题

  // // Required to use NIST 3GPP model
  
  // //install all platoonNodes with LTE devices
  // ////NetDevice
  // lteHelper->SetAttribute("UseSidelink",BooleanValue(true));
  // NetDeviceContainer platoonDev = lteHelper->InstallUeDevice (platoonNodes);

  // //Application 模块需要自己编写  适当简化 CBR(?)  
  // // m_factory.SetTypeId(VehicleApplication::GetTypeId());
  // // m_factory.SetAttribute(name,value);//(?)paper 中 set 有误
  // //对NodeContainer中的每个节点添加VehicleApplication对象 paper代码(?) 
  
  // //InternetStack
  // InternetStackHelper internet;
  // internet.Install (platoonNodes); 

  // //Assign IP Address
  // Ipv4InterfaceContainer platoonIpIface;
  // platoonIpIface = epcHelper->AssignIpv4Address(platoonDev);

  // //相对静态的Platoon模型的拓扑场景设计与实现
  //  Ptr<ListPositionAllocator>positionAlloc = CreateObject<ListPositionAllocator>();//创建ListPositionAllocator类型对象，用来存储车道上Platoon的位置
  //  Ptr<ExponentialRandomVariable>uv = CreateObject<ExponentialRandomVariable>(); //创建ExponentialRandomVariable类型对象，生成指数分布随机数（每队车之间的间隔）
  //  for(int i=0;i<nodeNum/2;i+=platoonLen){//设置platoon内部节点的间距
  //   positionAlloc->Add(Vector(preD,0.0,0.0));
  //   for(int j =1;j<platoonLen;j++){
  //       preD+=plantoonInterD;
  //       positionAlloc->Add(Vector(preD,0.0,0.0));
  //   }
  //   //platoon间距离
  //   preD+=uv->GetValue(expDistance,comDistance);
  // }
  
  // //创建Mobility
  // MobilityHelper mobility;
  // mobility.SetPositionAllocator(positionAlloc);
  // mobility.SetMobilityModel("ns:ConstantPositionMobilityModel");
  // mobility.Install(platoonNodes);

  // //移动模型的platoon模型的拓扑场景实现（市区or高速公路）->需要SUMO仿真辅助（暂时略）
  

}
