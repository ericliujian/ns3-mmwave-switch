#!/bin/bash
# This bash script is used to run the simulations for: 5g.cc

./waf configure 

#change bandwith to 1e7 for 4G LTE signal
bandwidth=1e7
#change frequency to 0.7, 0.85,1.7,1.9,2.1 for 4G LTE, 28 39 47 73.5 for 5G 39e9 47e9 73.5e9
frequency=5.9e9
interPacketInterval=30
speed=20

#three weather impacts, need to change in for loops
#particleradius=0.
#visibility=10
#humidity=0

intraGroupDistance=10


environment1="l" # the parameter defines the channel condition, l 
environment2="v" # the parameter defines the channel condition, v
environment3="n" # the parameter defines the channel condition, n  
commScenario1="V2V-Highway" # communication environment 
commScenario2="V2V-Urban" # communication environment  
commScenario3="Extended-V2V-Highway" # communication environment 
commScenario4="Extended-V2V-Urban" # communication environment 


while IFS=, read -r humidity v particleradius visibility;
do 
  ./waf --run "4g --bandwidth=$bandwidth --frequency=$frequency --iip=$interPacketInterval --speed=$speed --intraGroupDistance=$intraGroupDistance --Pvalue=$particleradius --Vvalue=$visibility --Hvalue=$humidity --channel_condition=$environment1 --scenario=$commScenario1"  
done < data/Blanding2021psaverage.csv



#three nested for loops works for setting up experiments for weather impacts, also add one for loop for frequency

<<comment
for frequency in 28e9 ; do
    for particleradius in $(seq 0.0000 0.00005 0.0009); do
        for visibility in $(seq 0.000 0.0005 0.005); do
            for humidity in $(seq 0 20 100); do
	        ./waf --run "5g --bandwidth=$bandwidth --frequency=$frequency --iip=$interPacketInterval --speed=$speed --intraGroupDistance=$intraGroupDistance --Pvalue=$particleradius --Vvalue=$visibility --Hvalue=$humidity --channel_condition=$environment1 --scenario=$commScenario1"            
            done
        done
    done
done


./waf --run "5g --bandwidth=$bandwidth --frequency=$frequency --iip=$interPacketInterval --speed=$speed --intraGroupDistance=$intraGroupDistance  --channel_condition=$environment1  --scenario=$commScenario2"
./waf --run "5g --bandwidth=$bandwidth --frequency=$frequency --iip=$interPacketInterval --speed=$speed --intraGroupDistance=$intraGroupDistance  --channel_condition=$environment1  --scenario=$commScenario3"
./waf --run "5g --bandwidth=$bandwidth --frequency=$frequency --iip=$interPacketInterval --speed=$speed --intraGroupDistance=$intraGroupDistance  --channel_condition=$environment1  --scenario=$commScenario4"
./waf --run "5g --bandwidth=$bandwidth --frequency=$frequency --iip=$interPacketInterval --speed=$speed --intraGroupDistance=$intraGroupDistance  --channel_condition=$environment2  --scenario=$commScenario1"
./waf --run "5g --bandwidth=$bandwidth --frequency=$frequency --iip=$interPacketInterval --speed=$speed --intraGroupDistance=$intraGroupDistance  --channel_condition=$environment2  --scenario=$commScenario2"
./waf --run "5g --bandwidth=$bandwidth --frequency=$frequency --iip=$interPacketInterval --speed=$speed --intraGroupDistance=$intraGroupDistance  --channel_condition=$environment2  --scenario=$commScenario3"
./waf --run "5g --bandwidth=$bandwidth --frequency=$frequency --iip=$interPacketInterval --speed=$speed --intraGroupDistance=$intraGroupDistance  --channel_condition=$environment2  --scenario=$commScenario4"
./waf --run "5g --bandwidth=$bandwidth --frequency=$frequency --iip=$interPacketInterval --speed=$speed --intraGroupDistance=$intraGroupDistance  --channel_condition=$environment3  --scenario=$commScenario1"
./waf --run "5g --bandwidth=$bandwidth --frequency=$frequency --iip=$interPacketInterval --speed=$speed --intraGroupDistance=$intraGroupDistance  --channel_condition=$environment3  --scenario=$commScenario2"
./waf --run "5g --bandwidth=$bandwidth --frequency=$frequency --iip=$interPacketInterval --speed=$speed --intraGroupDistance=$intraGroupDistance  --channel_condition=$environment3  --scenario=$commScenario3"
./waf --run "5g --bandwidth=$bandwidth --frequency=$frequency --iip=$interPacketInterval --speed=$speed --intraGroupDistance=$intraGroupDistance  --channel_condition=$environment3  --scenario=$commScenario4"
comment


