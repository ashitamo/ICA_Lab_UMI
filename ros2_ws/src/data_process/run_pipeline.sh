#!/bin/bash
# Usage: ./run_pipeline.sh <DEMO_PREFIX> <EXE_PREFIX>
# parameter 1: demo data folder with pkl file (eg: data/demo_5)
# parameter 2: exe data folder for output (eg: exe_5)

# --- 紀錄開始時間 ---
START_TIME=$(date +%s)

# --- 參數檢查 ---
if [ "$#" -ne 2 ]; then
    echo "使用方式: $0 <DEMO_PREFIX> <EXE_PREFIX>"
    echo "範例: $0 my_input_data my_output_dir"
    echo "    (這將使用 my_input_data6, my_input_data1, my_output_dir1, 等)"
    exit 1
fi

# --- 設定變數 ---
# $1 是第一個參數 (例如: demo_data 或 my_new_demo)
DEMO_PREFIX=$1
# $2 是第二個參數 (例如: exe_data 或 my_new_exe)
EXE_PREFIX=$2

# --- Clean the EXE_PREFIX folder ---
echo "--- 清理 ${EXE_PREFIX} 資料夾 ---"
if [ -d "${EXE_PREFIX}" ]; then
    rm -rf "${EXE_PREFIX}"/*
    echo "${EXE_PREFIX} 資料夾已清空。"
else
    echo "${EXE_PREFIX} 資料夾不存在，將在後續步驟中建立。"
fi


# --- 1. 設定環境變數 ---
echo "--- 步驟 1: 設定 OPENAI_API_KEY 環境變數 ---"

# --- 2. 執行 pkl_convert.py ---
# 使用 ${DEMO_PREFIX}6 取代 demo_data6
echo "--- 步驟 2: 執行 pkl_convert.py (使用 ${DEMO_PREFIX}) ---"
python3 pkl_convert.py ${DEMO_PREFIX}/demo.pkl

if [ $? -ne 0 ]; then
    echo "pkl_convert.py 執行失敗！停止腳本。"
    # 在失敗時也可以顯示目前已花費的時間（可選）
    END_TIME=$(date +%s)
    ELAPSED=$((END_TIME - START_TIME))
    echo "目前已花費時間：${ELAPSED} 秒"
    exit 1
fi

# --- 3. 執行 task_segmentation.py ---
# 使用 ${DEMO_PREFIX} 和 ${EXE_PREFIX} 取代 demo_data1 和 exe_data1
echo "--- 步驟 3: 執行 task_segmentation_threephase.py (使用 ${DEMO_PREFIX}, ${EXE_PREFIX}) ---"
python3 task_segmentation.py ${DEMO_PREFIX} ${EXE_PREFIX}

if [ $? -ne 0 ]; then
    echo "task_segmentation.py 執行失敗！停止腳本。"
    END_TIME=$(date +%s)
    ELAPSED=$((END_TIME - START_TIME))
    echo "目前已花費時間：${ELAPSED} 秒"
    exit 1
fi

# --- 4. 執行 task_manager_traject_smooth.py ---
echo "--- 步驟 4: 執行 task_manager_traject.py (使用 ${DEMO_PREFIX}, ${EXE_PREFIX}) ---"
python3 task_manager_traject_smooth.py ${DEMO_PREFIX} ${EXE_PREFIX}

if [ $? -ne 0 ]; then
    echo "task_manager_traject.py 執行失敗！停止腳本。"
    END_TIME=$(date +%s)
    ELAPSED=$((END_TIME - START_TIME))
    echo "目前已花費時間：${ELAPSED} 秒"
    exit 1
fi

# --- 計算並輸出總花費時間 ---
END_TIME=$(date +%s)
ELAPSED=$((END_TIME - START_TIME))

HOURS=$((ELAPSED / 3600))
MINUTES=$(((ELAPSED % 3600) / 60))
SECONDS=$((ELAPSED % 60))

printf -v ELAPSED_STR "%02d:%02d:%02d" "$HOURS" "$MINUTES" "$SECONDS"

echo "--- ✅ 所有流程已完成！ ---"
echo "總花費時間：${ELAPSED_STR} (約 ${ELAPSED} 秒)"
