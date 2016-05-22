#!/bin/bash

if [ $# != 1 ] ; then 
echo "USAGE: $0 inputpath" 
exit 1; 
fi 

if ! [ -d $1 ]
then
   echo "input path $1 not exist, exit"
   exit 1;
fi

input=$1
input="$input/*"

cd ~/ns3/ndnSIM/ns-3

for fileinput in $input
do
  ./waf --run "nrndn --accidentNum=200 --transRange=300 --method=0 --noFwStop=true --TTLMax=1 --interestFreq=0.1 --inputDir=$fileinput --outputDir=$fileinput"
  ./waf --run "nrndn --accidentNum=200 --transRange=300 --method=1 --noFwStop=true --TTLMax=1 --interestFreq=0.1 --inputDir=$fileinput --outputDir=$fileinput"
  ./waf --run "nrndn --accidentNum=200 --transRange=300 --method=2 --noFwStop=true --TTLMax=1 --interestFreq=0.1 --inputDir=$fileinput --outputDir=$fileinput"
done
