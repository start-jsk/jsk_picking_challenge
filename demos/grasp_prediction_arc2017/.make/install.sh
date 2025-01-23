#!/bin/bash

echo_bold () {
  echo -e "\033[1m$*\033[0m"
}

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set -e

# Setup Anaconda {{

echo_bold "==> Installing Anaconda"
if [ "$PYTHONVERSION" = "3" ]; then
  if [ ! -e $ROOT/.anaconda3 ]; then
    curl -L https://github.com/wkentaro/dotfiles/raw/f5151644cdb8a0503e8d1d33ad40ca5a7c73b14b/local/bin/install_anaconda3.sh | bash -s $ROOT
  fi
  ln -fs $ROOT/.anaconda3 $ROOT/.anaconda
else
  if [ ! -e $ROOT/.anaconda2 ]; then
    curl -L https://github.com/wkentaro/dotfiles/raw/f5151644cdb8a0503e8d1d33ad40ca5a7c73b14b/local/bin/install_anaconda2.sh | bash -s $ROOT
  fi
  ln -fs $ROOT/.anaconda2 $ROOT/.anaconda
fi
source $ROOT/.anaconda/bin/activate

# cupy-cudaXY does not work with Python3.7.Z
[[ $(conda list | egrep '^python ' | awk '{print $2}') =~ 3.7.* ]] && conda install -y python=3.6
echo "PYTHON_VERSION: $(conda list | egrep '^python ' | awk '{print $2}')"

echo "CONDA_VERSION: $(conda --version 2>&1 | awk '{print $2}')"
echo "CONDA_PREFIX: $CONDA_PREFIX"
echo "CONDA_DEFAULT_ENV: $CONDA_DEFAULT_ENV"

echo_bold "==> Installing latest pip and setuptools"
pip install -U pip setuptools wheel

# https://github.com/conda/conda/issues/448
# https://github.com/conda/conda/issues/7173
PYTHON_VERSION=$(python -c 'import sys; sys.stdout.write(sys.version[0:3])')
echo_bold "==> Patching $ROOT/.anaconda/lib/python$PYTHON_VERSION/site.py"
sed -i -e 's/ENABLE_USER_SITE = None/ENABLE_USER_SITE = False/g' $ROOT/.anaconda/lib/python$PYTHON_VERSION/site.py

# }}

# Install Requirements {{

echo_bold "==> Installing requirements for grasp_prediction_arc2017_lib"

# if [ "$PYTHONVERSION" = "2" ]; then
#   conda install -y pyqt
# fi
conda install -y pyqt  # for labelme and pyqt5 from pip doesn't work on travis

pip install 'cython>=0.23' opencv-python
pip install -r requirements.txt

# }} Install Requirements

# Install grasp_prediction_arc2017_lib
echo_bold "==> Installing grasp_prediction_arc2017_lib"
pip install -e .

set +e

echo_bold "\nAll is well! You can start using grasp_prediction_arc2017_lib!

  $ source $ROOT/.anaconda/bin/activate
  $ python -c 'import grasp_prediction_arc2017_lib'

You may also want to install following requirements:

  $ pip install cupy-cuda92  # cupy, cupy-cuda80, cupy-cuda90, or ...
  $ pip install chainermn
"
