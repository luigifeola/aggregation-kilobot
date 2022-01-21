#!/bin/bash

### How it works for me ###
# in ARGoS folder run the following:
# ./src/examples/experiments/aggregation_batch/aggregation_exp.sh /src/examples/experiments/aggregation_batch aggregation_experiment.argos

if [ "$#" -ne 2 ]; then
    echo "Usage: simple_experiment.sh (from src folder) <base_config_dir> <base_config_file_name>"
    exit 11
fi

wdir=`pwd`
base_config_s=.$1/$2
echo "base_config_s:" $base_config_s
if [ ! -e $base_config_s ]; then
    base_config_s=$wdir$1/$2
    if [ ! -e $base_config_s ]; then
        echo "Error: missing configuration file '$base_config_s'" 1>&2
        exit 1
    fi
fi

base_config_c=.$1/$3
if [ ! -e $base_config_c ]; then
    base_config_c=$wdir$1/$3
    if [ ! -e $base_config_c ]; then
        echo "Error: missing configuration file '$base_config_c'" 1>&2
        exit 1
    fi
fi

res_dir=$wdir/"results/aggregation"
if [[ ! -e $res_dir ]]; then
    cmake -E make_directory $res_dir
# else
#     echo "Error: directory '$res_dir' already exists" 
#     exit 1
fi



base_dir=`dirname $base_config_s`
# echo base_dir $base_dir
echo "$CONFIGURATION_FILE" | egrep "^$SHARED_DIR" &> /dev/null || exit 1

numrobots="20"

###################################
# experiment_length is in seconds #
###################################
experiment_length="1800"
date_time=`date "+%Y-%m-%d"`
RUNS=100

param_dir=$res_dir/"aggregation_"$date_time"_robots#"$numrobots"_"$experiment_length"#seconds"

if [[ ! -e $param_dir ]]; then
    cmake -E make_directory $param_dir
fi

for it in $(seq 1 $RUNS); do
    
    configs=`printf 'configs_nrob%d_timeout%03d_seed%03d.argos' $numrobots $it`
    # echo configs $configs
    cp $base_config_s $configs
    sed -i "s|__TIMEEXPERIMENT__|$experiment_length|g" $configs
    sed -i "s|__SEED__|$it|g" $configs
    sed -i "s|__NUMROBOTS__|$numrobots|g" $configs

    robot_positions_file="seed#${it}_kiloLOG.tsv"
    sed -i "s|__ROBPOSOUTPUT__|$robot_positions_file|g" $configs


    
    
    echo "Running next configuration seed $it Robots $numrobots"
    echo "argos3 -c $1$configs"
    argos3 -c './'$configs
    
    mv *.tsv $param_dir
    rm *.argos
done