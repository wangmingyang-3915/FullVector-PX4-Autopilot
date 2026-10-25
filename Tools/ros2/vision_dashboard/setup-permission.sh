#!/usr/bin/env bash
# One-time graphical authorization for exactly one systemctl restart command.
set -Eeo pipefail

readonly source_rule=/home/jetson/yahboom_ws/vision_dashboard/sudoers.vision-dashboard
readonly target_rule=/etc/sudoers.d/vision-dashboard

if sudo -n -l /usr/bin/systemctl restart apriltag-stack.service >/dev/null 2>&1; then
    zenity --info --title='视觉链路监控' --text='受限重启权限已经启用。现在可以直接在监控页面一键重启视觉链路。'
    exit 0
fi

if ! /usr/sbin/visudo -cf "$source_rule" >/dev/null 2>&1; then
    zenity --error --title='视觉链路监控' --text='权限配置文件语法检查失败，未进行安装。'
    exit 1
fi

if ! pkexec /usr/bin/install -o root -g root -m 0440 "$source_rule" "$target_rule"; then
    zenity --error --title='视觉链路监控' --text='授权未完成。监控页面仍可查看数据，但重启按钮暂不可用。'
    exit 1
fi

if sudo -n -l /usr/bin/systemctl restart apriltag-stack.service >/dev/null 2>&1; then
    zenity --info --title='视觉链路监控' --text='受限重启权限已启用。此权限只允许重启 apriltag-stack.service。'
else
    zenity --error --title='视觉链路监控' --text='规则已安装，但权限检查未通过。请检查 /etc/sudoers.d/vision-dashboard。'
    exit 1
fi
