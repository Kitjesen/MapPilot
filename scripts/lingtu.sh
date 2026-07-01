#!/bin/bash
# ============================================================
# lingtu.sh 鈥?MapPilot 缁熶竴鍚姩鍏ュ彛
#
# 鐢ㄦ硶:
#   ./lingtu.sh map     寤哄浘妯″紡锛堟墜鏌勯仴鎺?+ SLAM 寤哄浘锛?#   ./lingtu.sh save    淇濆瓨鍦板浘锛堣嚜鍔ㄧ敓鎴?.pcd + .pickle锛?#   ./lingtu.sh nav     瀵艰埅妯″紡锛堣嚜鍔ㄥ姞杞芥渶鏂板湴鍥撅級
#   ./lingtu.sh status  妫€鏌ョ郴缁熺姸鎬侊紙璇濋銆佸湴鍥俱€乀F锛?# ============================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LINGTU_PYTHON="${LINGTU_PYTHON:-python3}"
LINGTU_CLI="$WORKSPACE_DIR/lingtu.py"
TOMOGRAM_DIR="$WORKSPACE_DIR/src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/rsc/tomogram"
PCD_DIR="$WORKSPACE_DIR/src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/rsc/pcd"
MAP_DIR="$WORKSPACE_DIR/maps"

_run_lingtu() {
    cd "$WORKSPACE_DIR"
    exec "$LINGTU_PYTHON" "$LINGTU_CLI" "$@"
}

# 鈹€鈹€ 閫氱敤鍒濆鍖?鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
_init() {
    cd "$WORKSPACE_DIR"
    if [ ! -f "install/setup.bash" ]; then
        echo -e "${RED}閿欒: 鏈壘鍒?install/setup.bash锛岃鍏?make build${NC}"
        exit 1
    fi
    source install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE="$WORKSPACE_DIR/config/fastdds_no_shm.xml"
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
}

# 鈹€鈹€ 鑷姩妫€娴嬫渶鏂板湴鍥?鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
_find_latest_map() {
    local LATEST_PICKLE
    LATEST_PICKLE=$(ls -t "$TOMOGRAM_DIR"/*.pickle 2>/dev/null | head -1)
    if [ -n "$LATEST_PICKLE" ]; then
        echo "${LATEST_PICKLE%.pickle}"
        return
    fi
    local LATEST_PCD
    LATEST_PCD=$(ls -t "$PCD_DIR"/*.pcd 2>/dev/null | head -1)
    if [ -n "$LATEST_PCD" ]; then
        echo "${LATEST_PCD%.pcd}"
        return
    fi
    echo ""
}

# 鈹€鈹€ 瀛愬懡浠? map 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
cmd_map() {
    echo -e "${BOLD}${BLUE}鈻?寤哄浘妯″紡${NC}"
    echo -e "  濮旀墭褰撳墠 Module-first CLI: ${GREEN}python lingtu.py map${NC}"
    echo ""
    _run_lingtu map
}

# 鈹€鈹€ 瀛愬懡浠? save 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
cmd_save() {
    echo -e "${BOLD}${BLUE}鈻?淇濆瓨鍦板浘${NC}"
    _init
    mkdir -p "$MAP_DIR" "$PCD_DIR" "$TOMOGRAM_DIR"

    local TIMESTAMP MAP_NAME
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    MAP_NAME="map_${TIMESTAMP}"
    echo -e "  鍦板浘鍚嶇О: ${GREEN}${MAP_NAME}${NC}"
    echo ""

    # Step 1: 淇濆瓨 PCD
    echo -e "${GREEN}[1/3] 璋冪敤 SLAM 淇濆瓨鏈嶅姟...${NC}"
    # 鏈嶅姟鏌ユ壘椤哄簭锛?    #   /nav/save_map       鈥?slam_fastlio2.launch.py remap 鍚庣殑鏍囧噯鎺ュ彛锛堥閫夛級
    #   /pgo/save_maps      鈥?PGO 鑺傜偣鐩存帴鏆撮湶锛堟湁鍥炵幆浼樺寲鏃舵洿鍑嗭級
    #   /fastlio2/save_map  鈥?namespace 涓嬬殑 Fast-LIO2 鍘熷鏈嶅姟
    local SVC
    if ros2 service list 2>/dev/null | grep -q "^/nav/save_map$"; then
        SVC="/nav/save_map"
    elif ros2 service list 2>/dev/null | grep -q "^/pgo/save_maps$"; then
        SVC="/pgo/save_maps"
    elif ros2 service list 2>/dev/null | grep -q "^/fastlio2/save_map$"; then
        SVC="/fastlio2/save_map"
    else
        echo -e "${RED}閿欒: 鏈壘鍒板湴鍥句繚瀛樻湇鍔?{NC}"
        echo -e "${YELLOW}宸叉煡鎵? /nav/save_map, /pgo/save_maps, /fastlio2/save_map${NC}"
        echo -e "${YELLOW}璇风‘璁?./lingtu.sh map 鐨勮妭鐐逛粛鍦ㄨ繍琛?{NC}"
        exit 1
    fi
    ros2 service call "$SVC" interface/srv/SaveMaps \
        "{file_path: '$MAP_DIR/$MAP_NAME', save_patches: false}"
    echo -e "${GREEN}鉁?PCD 宸蹭繚瀛? $MAP_DIR/${MAP_NAME}.pcd${NC}"

    # Step 2: 澶嶅埗鍒?PCT 宸ヤ綔鐩綍
    echo -e "${GREEN}[2/3] 澶嶅埗鍒拌鍒掑櫒宸ヤ綔鐩綍...${NC}"
    if [ ! -f "$MAP_DIR/${MAP_NAME}.pcd" ]; then
        echo -e "${RED}閿欒: PCD 鏂囦欢鏈敓鎴愶紝璇锋鏌?SLAM 鏈嶅姟杈撳嚭${NC}"
        exit 1
    fi
    cp "$MAP_DIR/${MAP_NAME}.pcd" "$PCD_DIR/${MAP_NAME}.pcd"
    ln -sf "$PCD_DIR/${MAP_NAME}.pcd" "$PCD_DIR/latest.pcd"
    echo -e "${GREEN}鉁?宸插鍒跺埌 $PCD_DIR/${NC}"

    # Step 3: 鐢熸垚 Tomogram
    echo -e "${GREEN}[3/3] 鐢熸垚 PCT Tomogram锛?~5 鍒嗛挓锛?..${NC}"
    cd "$WORKSPACE_DIR/src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/tomography/scripts"
    if python3 tomography.py --scene "$MAP_NAME"; then
        ln -sf "$TOMOGRAM_DIR/${MAP_NAME}.pickle" "$TOMOGRAM_DIR/latest.pickle"
        echo ""
        echo -e "${BOLD}${GREEN}鉁?鍦板浘淇濆瓨瀹屾垚锛?{NC}"
        echo -e "  PCD:      ${BLUE}$PCD_DIR/${MAP_NAME}.pcd${NC}"
        echo -e "  Tomogram: ${BLUE}$TOMOGRAM_DIR/${MAP_NAME}.pickle${NC}"
        echo ""
        echo -e "${YELLOW}涓嬩竴姝? ${GREEN}./lingtu.sh nav${NC}"
    else
        echo -e "${RED}閿欒: Tomogram 鐢熸垚澶辫触${NC}"
        exit 1
    fi
}

# 鈹€鈹€ 瀛愬懡浠? nav 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
cmd_nav() {
    echo -e "${BOLD}${BLUE}鈻?瀵艰埅妯″紡${NC}"
    _init

    local MAP_PATH
    MAP_PATH=$(_find_latest_map)
    if [ -z "$MAP_PATH" ]; then
        echo -e "${RED}閿欒: 鏈壘鍒板湴鍥炬枃浠?{NC}"
        echo -e "${YELLOW}璇峰厛杩愯: ${GREEN}./lingtu.sh map${YELLOW} + ${GREEN}./lingtu.sh save${NC}"
        exit 1
    fi

    local MAP_BASE
    MAP_BASE=$(basename "$MAP_PATH")
    echo -e "  浣跨敤鍦板浘: ${GREEN}${MAP_BASE}${NC}"
    if [[ "$MAP_PATH" == *.pickle ]]; then
        echo -e "  ${YELLOW}娉ㄦ剰: 鏈壘鍒?.pickle锛屽皢浠?.pcd 瀹炴椂鏋勫缓 Tomogram锛堣緝鎱級${NC}"
    fi
    echo ""

    if [ -f "${MAP_PATH}.pickle" ]; then
        _run_lingtu nav --tomogram "${MAP_PATH}.pickle"
    fi
    export NAV_MAP_PATH="$MAP_PATH"
    echo -e "  濮旀墭褰撳墠 Module-first CLI: ${GREEN}python lingtu.py nav${NC}"
    _run_lingtu nav
}

# 鈹€鈹€ 瀛愬懡浠? status 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
cmd_status() {
    echo -e "${BOLD}${BLUE}鈻?绯荤粺鐘舵€佹鏌?{NC}"
    echo -e "  濮旀墭褰撳墠 Module-first CLI: ${GREEN}python lingtu.py status${NC}"
    echo ""
    _run_lingtu status
}

# 鈹€鈹€ 甯姪 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
cmd_help() {
    echo -e "${BOLD}lingtu.sh 鈥?MapPilot 缁熶竴鍚姩鍏ュ彛${NC}"
    echo ""
    echo -e "  ${GREEN}./lingtu.sh map${NC}     鍏煎鍏ュ彛锛屽鎵?python lingtu.py map"
    echo -e "  ${GREEN}./lingtu.sh save${NC}    淇濆瓨鍦板浘锛圥CD + Tomogram 鍏ㄨ嚜鍔級"
    echo -e "  ${GREEN}./lingtu.sh nav${NC}     鍏煎鍏ュ彛锛屽鎵?python lingtu.py nav"
    echo -e "  ${GREEN}./lingtu.sh status${NC}  鍏煎鍏ュ彛锛屽鎵?python lingtu.py status"
    echo ""
    echo -e "${YELLOW}鍏稿瀷娴佺▼:${NC}"
    echo -e "  1. ${GREEN}./lingtu.sh map${NC}   閬ユ帶鏈哄櫒浜哄缓鍥?
    echo -e "  2. Ctrl+C 鍋滄寤哄浘"
    echo -e "  3. ${GREEN}./lingtu.sh save${NC}  淇濆瓨鍦板浘锛堢瓑寰?2~5 鍒嗛挓锛?
    echo -e "  4. ${GREEN}./lingtu.sh nav${NC}   鍚姩鑷富瀵艰埅"
    echo ""
}

# 鈹€鈹€ 鍏ュ彛 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
case "${1:-help}" in
    map)    cmd_map    ;;
    save)   cmd_save   ;;
    nav)    cmd_nav    ;;
    status) cmd_status ;;
    *)      cmd_help   ;;
esac
