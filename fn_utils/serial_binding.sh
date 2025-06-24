#!/bin/bash
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
export PATH

# 定义规则文件路径
rules_file="/etc/udev/rules.d/95-persistent-serial.rules"

# 初始化规则内容
rules_header='ACTION!="add", GOTO="persistent_serial_end"
SUBSYSTEM!="tty", GOTO="persistent_serial_end"
KERNEL!="ttyUSB[0-9]*|ttyACM[0-9]*", GOTO="persistent_serial_end"

'
rules_footer='\nLABEL="persistent_serial_end"'
rules_content="$rules_header"

# 查找所有串口设备
devices=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sort -V)

# 检查设备是否存在
if [ -z "$devices" ]; then
    echo "未找到任何可用的串口设备！"
    exit 1
fi

# 显示设备列表
echo "发现以下可用的串口设备："
echo "------------------------"
i=1
for dev in $devices; do
    # 判断设备类型
    case $dev in
        *ttyUSB*) type="USB";;
        *ttyACM*) type="ACM";;
        *) type="UNKNOWN";;
    esac
    
    echo "[$i] $dev ($type)"
    eval "device_$i=\"$dev\""
    i=$((i + 1))
done
echo "------------------------"
echo "提示：当您绑定完设备之后，原来的规则文件将被覆盖"
echo ""

# 遍历所有设备
idx=1
while [ $idx -lt $i ]; do
    current_dev=$(eval "echo \$device_$idx")
    
    # 询问用户是否绑定
    while true; do
        printf "是否要绑定设备 %s？(y/n) " "$current_dev"
        read yn
        case $yn in
            [Yy] ) bind=1; break;;
            [Nn] ) bind=0; break;;
            * ) echo "请输入 y 或 n";;
        esac
    done

    if [ $bind -eq 1 ]; then
        # 提取KERNELS值
        kernels_value=$(udevadm info -a -n "$current_dev" | grep 'KERNELS=="' | head -n2 | tail -n1 | sed -E 's/.*KERNELS=="([^"]+)".*/\1/')
        
        # 获取设备别名
        echo "正在配置设备：$current_dev"
        printf "请输入设备别名（默认：%s）：" "${current_dev##*/}"
        read symlink
        symlink=${symlink:-${current_dev##*/}}

        # 自动设置参数
        owner="$USER"
        mode="0666"

        # 生成规则行
        rule_line="KERNELS==\"$kernels_value\"         ,SYMLINK=\"$symlink\" ,OWNER=\"$owner\",    MODE=\"$mode\"\n"
        rules_content="$rules_content$rule_line"
    fi

    idx=$((idx + 1))
done

# 写入规则文件
echo "正在生成udev规则文件..."
printf '%b' "$rules_content$rules_footer" | sudo tee "$rules_file" >/dev/null

# 重新加载udev规则
echo "重新加载udev规则..."
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "操作完成！当前绑定规则：/etc/udev/rules.d/95-persistent-serial.rules"
echo "配置完成！"

while true; do
    printf "是否立即重启计算机以使更改生效？(yes/no) "
    read reboot_choice
    case $reboot_choice in
        [Yy][Ee][Ss] ) 
            sudo reboot 
            exit 0
            ;;
        [Nn][Oo] ) 
            echo "您可以稍后手动重启计算机使更改生效"
            exit 0
            ;;
        * ) 
            echo "请输入 yes 或 no"
            ;;
    esac
done
