#!/bin/bash
./metricEvaluator/metricEvaluator -s ./$1/$1.result -r ./$1/$1.relations -w "{1.0,1.0,1.0,0.0,0.0,0.0}" -e ./$1/$1-$2_trans_error.log -o ./$1/$1.log 
./metricEvaluator/metricEvaluator -s ./$1/$1.result -r ./$1/$1.relations -w "{0.0,0.0,0.0,1.0,1.0,1.0}" -e ./$1/$1-$2_rot_error.log -o ./$1/$1.log
# ./metricEvaluator/metricEvaluator -q -s ./$1/$1.result -r ./$1/$1.relations -w "{1.0,1.0,1.0,0.0,0.0,0.0}" -e ./$1/$1_sq_trans_error.log -o ./$1/$1.log 
# ./metricEvaluator/metricEvaluator -q -s ./$1/$1.result -r ./$1/$1.relations -w "{0.0,0.0,0.0,1.0,1.0,1.0}" -e ./$1/$1_sq_rot_error.log -o ./$1/$1.log 
