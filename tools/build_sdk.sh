#!/bin/sh

declare -a target_list=(
  "nm180100"
  "nm180110"
)

variant="Release"
f_lorawan=false
f_ble=false
f_mesh=false
f_hal=false
f_os=false
target_flag=false
target_valid=false
target_name="None"

display_help () {
  echo "Usage: build_sdk.sh [OPTION] -t TARGET"
  echo "  -r    build Release variant"
  echo "  -d    build Debug variant"
  echo "  -l    build LoRaWAN"
  echo "  -b    build BLE"
  echo "  -m    build LoRa mesh"
  echo "  -O    build OS"
  echo "  -H    build HAL"
  echo "  -t    valid TARGET values:"
  for target in "${target_list[@]}"
  do
    echo "          $target"
  done
}

while getopts "rdlbmHOt:" flag; do
  case ${flag} in
    r)
      variant="Release"
      ;;
    d)
      variant="Debug"
      ;;
    l)
      f_lorawan=true
      ;;
    b)
      f_ble=true
      ;;
    m)
      f_mesh=true
      ;;
    O)
      f_os=true
      ;;
    H)
      f_hal=true
      ;;
    t)
      target_flag=true
      target_name=$OPTARG
      ;;
    ?)
      display_help
      exit 1
      ;;
  esac
done

if ( ! $target_flag ) then
  echo "Missing -t target specification."
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

if ( ! $target_valid ) then
  echo "Invalid target specified"
  echo ""
  display_help
  exit 1
fi

SCRIPT_DIR=$(cd $(dirname -- "${BASH_SOURCE[0]}") && pwd)
ROOT_DIR=$(cd ${SCRIPT_DIR}/.. && pwd)
BUILD_DIR=${ROOT_DIR}/build

cd ${ROOT_DIR}

if [ -d ${BUILD_DIR} ]; then
  rm -rf ${BUILD_DIR}
fi

cmake \
  -DCMAKE_BUILD_TYPE=$variant \
  -DNM_TARGET=$target_name \
  -DFEATURE_RAT_LORAWAN_ENABLE=$f_lorawan \
  -DFEATURE_RAT_BLE_ENABLE=$f_ble \
  -DFEATURE_RAT_LORA_MESH_ENABLE=$f_mesh \
  -DBUILD_LORAWAN=$f_lorawan \
  -DBUILD_BLE=$f_ble \
  -DBUILD_LORA_MESH=$f_mesh \
  -DBUILD_RTOS=$f_os \
  -DBUILD_HAL=$f_hal \
  -S . -B ${BUILD_DIR} -G "Unix Makefiles"
if [ $? -ne 0 ]; then
  echo "Configuration error"
  exit 1
fi
echo "$target_name $variant Config Completed"

cmake --build ${BUILD_DIR}
if [ $? -ne 0 ]; then
  echo "Build error"
  exit 1
fi
echo "$target_name $variant Build Completed"

cmake --install ${BUILD_DIR}
if [ $? -ne 0 ]; then
  echo "Install error"
  exit 1
fi
echo "$target_name $variant Installation Completed"