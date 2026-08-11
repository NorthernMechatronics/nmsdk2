#!/bin/sh

function display_help () {
  echo "Usage: build_sdk.sh [OPTIONS]"
  echo "  --noclean   keep the existing build directory"
  echo "  --cmsis     build CMSIS libraries for family: <nm1801xx|cortex-m55>"
  echo "  --all       build all libraries"
  echo "  --lorawan   build LoRaWAN"
  echo "  --ble       build BLE"
  echo "  --mesh      build LoRa mesh"
  echo "  --os        build OS"
  echo "  --hal       build HAL"
  echo "  --tflm      build TFLM"
  echo "  --variant   override build variant: Debug or Release (default)"
  echo "  --target    valid TARGET values:"
  for target in "${target_list[@]}"
  do
    echo "              $target"
  done
}

function build_cmsis () {
  local local_family_name=$1
  local local_family_common_dir="${ROOT_DIR}/targets/${local_family_name}/common"
  local local_cmsis_dir="${local_family_common_dir}/cmsis"
  local local_cmsis_dsp_dir="${local_cmsis_dir}/CMSIS-DSP"
  local local_cmsis_nn_dir="${local_cmsis_dir}/CMSIS-NN"

  cd ${local_cmsis_dsp_dir}

  BUILD_DIR=${local_cmsis_dsp_dir}/build
  if [ $clean_build ]; then
    if [ -d ${BUILD_DIR} ]; then
      rm -rf ${BUILD_DIR}
    fi
  fi

  cmake -S . -B ${BUILD_DIR} -G "Unix Makefiles"
  if [ $? -ne 0 ]; then
    echo "Configuration error"
    exit 1
  fi
  echo "CMSIS DSP Config Completed"

  cmake --build ${BUILD_DIR} -j$numthread
  if [ $? -ne 0 ]; then
    echo "Build error"
    exit 1
  fi
  echo "CMSIS DSP Build Completed"

  cmake --install ${BUILD_DIR}
  if [ $? -ne 0 ]; then
    echo "Install error"
    exit 1
  fi
  echo "CMSIS DSP Installation Completed"

  cd ${local_cmsis_nn_dir}

  BUILD_DIR=${local_cmsis_nn_dir}/build
  if [ $clean_build ]; then
    if [ -d ${BUILD_DIR} ]; then
      rm -rf ${BUILD_DIR}
    fi
  fi

  cmake -S . -B ${BUILD_DIR} -G "Unix Makefiles"
  if [ $? -ne 0 ]; then
    echo "Configuration error"
    exit 1
  fi
  echo "CMSIS NN Config Completed"

  cmake --build ${BUILD_DIR} -j$numthread
  if [ $? -ne 0 ]; then
    echo "Build error"
    exit 1
  fi
  echo "CMSIS NN Build Completed"

  cmake --install ${BUILD_DIR}
  if [ $? -ne 0 ]; then
    echo "Install error"
    exit 1
  fi
  echo "CMSIS NN Installation Completed"
}

function build_target () {
  local local_target_name=$1
  echo "Building target: $local_target_name"

  BUILD_DIR=${ROOT_DIR}/build/$local_target_name

  cd ${ROOT_DIR}

  if [ $clean_build ]; then
    if [ -d ${BUILD_DIR} ]; then
      rm -rf ${BUILD_DIR}
    fi
  fi

  mkdir -p ${BUILD_DIR}
  cd ${BUILD_DIR}

  cmake \
    -DCMAKE_BUILD_TYPE=$variant \
    -DNM_TARGET=$local_target_name \
    -DFEATURE_RAT_LORAWAN_ENABLE=$f_lorawan \
    -DFEATURE_RAT_BLE_ENABLE=$f_ble \
    -DFEATURE_RAT_LORA_MESH_ENABLE=$f_mesh \
    -DFEATURE_TFLM_ENABLE=$f_tflm \
    -DBUILD_LORAWAN=$f_lorawan \
    -DBUILD_BLE=$f_ble \
    -DBUILD_LORA_MESH=$f_mesh \
    -DBUILD_RTOS=$f_os \
    -DBUILD_HAL=$f_hal \
    -DBUILD_TFLM=$f_tflm \
    -S ../../ -B . -G "Unix Makefiles"
  if [ $? -ne 0 ]; then
    echo "Configuration error"
    exit 1
  fi
  echo "$local_target_name $variant Config Completed"

  cmake --build . -j$numthread
  if [ $? -ne 0 ]; then
    echo "Build error"
    exit 1
  fi
  echo "$local_target_name $variant Build Completed"

  cmake --install .
  if [ $? -ne 0 ]; then
    echo "Install error"
    exit 1
  fi
  echo "$local_target_name $variant Installation Completed"
}

declare -a family_list=(
  "nm1801xx"
  "cortex-m55"
)

declare -a target_list=(
  "nm180100"
  "nm180110"
  "apollo510"
  "all"
)

OPTIONS=$(getopt --long cmsis:, all, ble, mesh, lorawan, hal, os, tflm, variant:, target: -- "$@")

variant="Release"

f_cmsis=false
f_tflm=false
f_lorawan=false
f_ble=false
f_mesh=false
f_hal=false
f_os=false
target_flag=false
target_valid=false
target_name="None"
family_name="None"
single_target=true
clean_build=true

let numthread=$(nproc)/2

if [ $numthread -lt 1 ]; then
  numthread=1
fi

if [ $# -eq 0 ]; then
  display_help
  exit 1
fi

while [ $1 ]; do
  case "$1" in
    --help)
      display_help
      exit 0
      ;;

    --noclean)
      echo "Keep the build directory"
      clean_build=true
      shift
      ;;

    --all)
      echo "Enable all features"
      f_tflm=true
      f_lorawan=true
      f_ble=true
      f_mesh=true
      f_hal=true
      f_os=true
      shift
      ;;

    --tflm)
      echo "Enable TFLM"
      f_tflm=true
      shift
      ;;

    --ble)
      echo "Enable BLE"
      f_ble=true
      shift
      ;;

    --lorawan)
      echo "Enable LoRaWAN"
      f_lorawan=true
      shift
      ;;

    --mesh)
      echo "Enable Mesh"
      f_mesh=true
      shift
      ;;

    --hal)
      echo "Enable HAL"
      f_hal=true
      shift
      ;;

    --os)
      echo "Enable OS"
      f_os=true
      shift
      ;;

    --variant)
      variant=$2
      echo "Variant set to $variant"
      shift 2
      ;;

    --target)
      target_flag=true
      if [ "$#" -lt 2 ]; then
        echo "Missing target name"
        exit 1
      else
        target_name=$2
        echo "Target set to $target_name"
        shift 2
      fi
      ;;

    --cmsis)
      f_cmsis=true
      if [ "$#" -lt 2 ]; then
        echo "Missing family name for CMSIS"
        exit 1
      else
        family_name=$2
        echo "Family set to $family_name"
        shift 2
      fi
      ;;

    *)
      display_help
      exit 1
      ;;
  esac
done

SCRIPT_DIR=$(cd $(dirname -- "${BASH_SOURCE[0]}") && pwd)
ROOT_DIR=$(cd ${SCRIPT_DIR}/.. && pwd)

if [ $f_cmsis = true ]; then
  for family in "${family_list[@]}"
  do
    if [ "$family_name" = "$family" ]; then
      build_cmsis $family_name
      exit 0
    fi
  done
  exit 1
fi

if [ $target_flag = false ]; then
  echo "Missing target specification."
  echo ""
  display_help
  exit 1
fi

for target in "${target_list[@]}"
do
  if [ "$target_name" = "$target" ]; then
    target_valid=true
  fi
done

if [ "$target_name" = "all" ]; then
  single_target=false
fi

if [ $target_valid = false ]; then
  echo "Invalid target specified"
  echo ""
  display_help
  exit 1
fi

if [ $single_target = false ]; then
  for target in "${target_list[@]}"
  do
    if [ "$target" != "all" ]; then
      build_target $target
    fi
  done
else
  build_target $target_name
fi

exit 0