# v-net


#2.preparation

gcc/g++ version should >= 4.8

boost libraries should be installed on the system:

###For Ubuntu
	12.04
		sudo aptitude install libboost1.48-all-dev
	12.10, 13.04, and newer versions
		sudo aptitude install libboost-all-dev

###For Fedora (for Fedora 18 and later only):
	sudo yum install boost-devel

###For MacOS (macports):
	sudo port instal boost

#3.configuration

C++11 standard are required

firstly,enter the ns-3 directory:


    cd ndn/ns-3/


To use the ns3,please configure first:

    CXXFLAGS="-Wall -g -std=c++11" ./waf -d debug --enable-examples configure

If boost library needs to specify, try:

    CXXFLAGS="-Wall -g -std=c++11" ./waf -d debug --enable-examples --boost-includes=/usr/local/include --boost-libs=/usr/local/lib configure

(Refer to http://ndnsim.net/2.0/faq.html#boost-libraries)

Then build the ns3 project with

    ./waf 

At last you can enjoy the ns3,
	
    ./waf --run=<your progremma>

example: 
    
    ./waf --run=nrndn_test
	./waf --run "nrndn_test_20160716_backup --method=0" < csma-multicast-1-0.pcap
	./waf --run "nrndn_20160126_easonOriginal --method=0" < csma-multicast-1-0.pcap

#4.Notes for Commands

##4.1	To run scenario and see what is happening, use the following command:
	NS_LOG=ndn.Consumer:ndn.Producer ./waf --run=nrndn

##4.2	Useful prefix:
	NS_LOG="ndn.nrndn.nrConsumer=level_debug|prefix_time|prefix_node|prefix_func:ndn.nrndn.nrProducer=level_debug|prefix_time|prefix_node|prefix_func" ./waf --run="nrndn --accidentNum=10"

##4.3
To run the nrndn method, use

	NS_LOG="ndn.nrndn.nrConsumer=level_debug|prefix_time|prefix_node|prefix_func:ndn.nrndn.nrProducer=level_debug|prefix_time|prefix_node|prefix_func" ./waf --run="nrndn --accidentNum=10"

To run the dist method, use

	NS_LOG="ndn.nrndn.tradConsumer=level_debug|prefix_time|prefix_node|prefix_func:ndn.nrndn.nrProducer=level_debug|prefix_time|prefix_node|prefix_func" ./waf --run="nrndn --accidentNum=10 --method=1"

To run the CDS method, use

	NS_LOG="ndn.nrndn.tradConsumer=level_debug|prefix_time|prefix_node|prefix_func:ndn.nrndn.nrProducer=level_debug|prefix_time|prefix_node|prefix_func" ./waf --run="nrndn --accidentNum=10 --method=2"

 
Good luck!
