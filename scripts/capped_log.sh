#!/usr/bin/env bash
# ============================================================
# 带上限的日志写入器 (防失控日志写爆磁盘)
# 用法: some_cmd 2>&1 | bash capped_log.sh <logfile> [max_mb=50]
# 行为: 持续消费 stdin (不会因停止写入而 SIGPIPE 杀掉上游进程),
#       写入达到上限后仅追加一条提示, 不再写盘。
# ============================================================
FILE="${1:?usage: capped_log.sh <logfile> [max_mb]}"
MAX_MB="${2:-50}"
awk -v file="$FILE" -v maxb="$((MAX_MB * 1024 * 1024))" '
BEGIN { w = 0; capped = 0 }
{
    if (w < maxb) {
        print >> file; w += length($0) + 1; fflush(file)
    } else if (!capped) {
        capped = 1
        print "[capped_log] 已达上限 " int(maxb/1048576) "MB, 停止写入 (上游进程继续运行)" >> file
        fflush(file)
    }
}'
