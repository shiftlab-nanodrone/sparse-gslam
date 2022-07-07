#!/bin/bash
wget -O aces/aces.log http://ais.informatik.uni-freiburg.de/slamevaluation/datasets/aces.clf
wget -O aces/aces.relations http://ais.informatik.uni-freiburg.de/slamevaluation/datasets/aces.relations

wget -O intel-lab/intel-lab.log http://ais.informatik.uni-freiburg.de/slamevaluation/datasets/intel.clf
wget -O intel-lab/intel-lab.relations http://ais.informatik.uni-freiburg.de/slamevaluation/datasets/intel.relations

wget -O mit-killian/mit-killian.log http://ais.informatik.uni-freiburg.de/slamevaluation/datasets/mit-killian.clf
wget -O mit-killian/mit-killian.relations http://ais.informatik.uni-freiburg.de/slamevaluation/datasets/mit-killian.relations

wget -O mit-csail/mit-csail.log http://ais.informatik.uni-freiburg.de/slamevaluation/datasets/mit-csail.clf
wget -O mit-csail/mit-csail.relations http://ais.informatik.uni-freiburg.de/slamevaluation/datasets/mit-csail.relations

cd usc-sal
wget -O usc-sal.log.gz https://dspace.mit.edu/bitstream/handle/1721.1/62281/run02.log.gz
gzip -d -f usc-sal.log.gz
cd ..

cd nsh_level_a
wget -O nsh_level_a.log.gz https://dspace.mit.edu/bitstream/handle/1721.1/62266/nsh_level_a.log.gz
gzip -d -f nsh_level_a.log.gz
cd ..

cd stanford-gates
wget -O stanford-gates.log.gz https://dspace.mit.edu/bitstream/handle/1721.1/62252/stanford-gates1.log.gz
gzip -d stanford-gates.log.gz
cd ..

cd intel-oregon
wget -O intel-oregon.log.gz https://dspace.mit.edu/bitstream/handle/1721.1/62253/intel-01-sorted.log.gz
gzip -d intel-oregon.log.gz
cd ..

cd albertb
wget -O albertb.log.gz https://dspace.mit.edu/bitstream/handle/1721.1/62265/albertb.sm.log.gz
gzip -d albertb.log.gz
cd ..

rm -rf metricEvaluator
git clone https://github.com/shiftlab-nanodrone/slam-metric-evaluator metricEvaluator
cd metricEvaluator
make -j4
cd ..