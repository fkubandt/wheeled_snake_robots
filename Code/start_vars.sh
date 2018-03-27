#!/bin/bash

## Author of original Script: Michael Nowak
## Modified by Frederike Kubandt
## This script automates the testing for different parameters in LpzRobots
## and generates gnuplot output files

## It is located in the same folder as the controller
## for another path './' needs to be changed

## Names of files and variables need to be changed for personal settings


##rm ./Makefile.depend		#remove Makefile.depend to make with new parameters when error occurs
LC_NUMERIC=en_US.UTF-8      #make sure, that the sequences use dots as decimal seperators
listVarIncline="`seq 0.00 0.005 0.07`" # seq BEGIN STEP END
listModes="`seq 0 2 1`"
listForces="`seq 00.09 0.001 0.10`"
listVarK="`seq -34 0.1 -30`"
now=$(date +"%d_%b_%Y_%H_%M_%S")

read -p "FORCEVAR(1)|INCLVAR(2)|KVAR(3)|KVARAIR(4) (CHECK ENVIRONMENT!!): " option

case "${option}" in
  *1*) folder="forcevariations"
     mkdir ../simulations ../simulations/sim_${now} ../simulations/sim_${now}/${folder} ../simulations/sim_${now}/${folder}/log ../simulations/sim_${now}/runfiles  ../simulations/sim_${now}/executables
	 cp ../simulations/Force.py ../simulations/sim_${now}/${folder}/
	 cp -r * ../simulations/sim_${now}/runfiles/ && cd ../simulations/sim_${now}/runfiles/ 
    
	  rm main2.o 
	  make param=1 mode=0 slope=0 force=0 kvar=0
	  ./start -f 5 mode0 -nographics -simtime 3
	  mv mode0Car.log ../${folder}/log
	  cp start ../executables/start_mode0
    for f in ${listForces}; do 
	    fDec=$(printf '%0.6f\n' $f)
		rm couplingrodneuron.o main2.o
		make param=1 mode=2 slope=0 force=${f} kvar=0
		./start -f 5 force${f} -nographics -simtime 3
		mv force${f}Car.log ../${folder}/log
		cp start ../executables/start_force${f}
    done
    ;;
  *2*)
    folder="incline"
	mkdir ../simulations ../simulations/sim_${now} ../simulations/sim_${now}/${folder} ../simulations/sim_${now}/${folder}/log ../simulations/sim_${now}/runfiles  ../simulations/sim_${now}/executables #create the desired folders
    cp ../simulations/Inclined.py ../simulations/sim_${now}/${folder}/
	cp -r * ../simulations/sim_${now}/runfiles/ && cd ../simulations/sim_${now}/runfiles/  # copy all simulation files into the folder and go there
 
    rm couplingrodneuron.o


	for i in ${listVarIncline}; do
		iDec=$(printf '%0.6f\n' $i)
		for m in ${listModes}; do
		    rm ./main2.o
			make param=1 mode=${m} slope=${i} force=0.205 kvar=0
##			name=
			./start -f 5 ${m}_${i} -nographics -simtime 15    #5 minuten Simulation jeweils
			mv ${m}_${i}Car.log ../${folder}/log
			cp start ../executables/start_${m}_${i}
		done
	done
    python3 ../Inclined.py
    ;;
  *3*)
	folder="synchrony_kvar"
	mkdir ../simulations ../simulations/sim_${now} ../simulations/sim_${now}/${folder} ../simulations/sim_${now}/${folder}/log ../simulations/sim_${now}/runfiles  ../simulations/sim_${now}/executables #create the desired folders
    
	cp -r * ../simulations/sim_${now}/runfiles/ && cd ../simulations/sim_${now}/runfiles/  # copy all simulation files into the folder and go there
 
    rm couplingrodneuron.o


	for i in ${listVarK}; do
		iDec=$(printf '%0.6f\n' $i)
		rm ./main2.o
		rm ./couplingrodneuron.o
		make param=1 mode=0 slope=0 force=0.097 kvar=${i}
##			name=
		./start -r 1335 -f 5 kvar_${i} -nographics -simtime 1    # i minuten Simulation jeweils , -r random seed
		mv kvar_${i}Car.log ../${folder}/log
##		cp start ../executables/start_${m}_${i}
	done
	;;
  *4*)
	folder="synchrony_kvar_air"
	mkdir ../simulations ../simulations/sim_${now} ../simulations/sim_${now}/${folder} ../simulations/sim_${now}/${folder}/log ../simulations/sim_${now}/runfiles  ../simulations/sim_${now}/executables #create the desired folders
    
	cp -r * ../simulations/sim_${now}/runfiles/ && cd ../simulations/sim_${now}/runfiles/  # copy all simulation files into the folder and go there
 
    rm couplingrodneuron.o


	for i in ${listVarK}; do
		iDec=$(printf '%0.6f\n' $i)
		rm ./main2.o
		make param=1 mode=0 slope=0 force=0.097 kvar=${i}
##			name=
		./start -r 1994 -f 5 kvar_${i}_air -nographics -simtime 3    #3 minuten Simulation jeweils , -r random seed
		mv kvar_${i}_airCar.log ../${folder}/log
		cp start ../executables/start_${m}_${i}
	done
	;;
esac
exit 0