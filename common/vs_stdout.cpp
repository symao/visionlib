#include "vs_stdout.h"
#include <stdlib.h>
#include <stdio.h>
#include <fstream>


#ifdef WIN32
#include <Windows.h>

void coutColor( const std::string& info,int color/*=WHITE*/ )
{
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),color);
    std::cout<<info<<std::endl;
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),COLOR_WHITE);
}

void coutError(const std::string& module,const std::string& info)
{
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),COLOR_RED);
    printf("[ERROR] ");
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),COLOR_DARKRED);
    std::cout<<module<<":"<<info<<std::endl;
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),COLOR_WHITE);

}
void coutWarn(const std::string& module,const std::string& info)
{
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),COLOR_GREEN);
    printf("[WARN] ");
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),COLOR_DARKGREEN);
    std::cout<<module<<":"<<info<<std::endl;
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),COLOR_WHITE);
}



#else
/*常用的ANSI控制码如下（有些不支持）：
    \033[0m 关闭所有属性
    \033[1m 高亮
    \033[2m 亮度减半
    \033[3m 斜体
    \033[4m 下划线
    \033[5m 闪烁
    \033[6m 快闪
    \033[7m 反显
    \033[8m 消隐
    \033[9m 中间一道横线
    10-19 关于字体的
    21-29 基本与1-9正好相反
    30-37 设置前景色
    40-47 设置背景色
    30:黑
    31:红
    32:绿
    33:黄
    34:蓝色
    35:紫色
    36:深绿
    37:白色
    38 打开下划线,设置默认前景色
    39 关闭下划线,设置默认前景色
    40 黑色背景
    41 红色背景
    42 绿色背景
    43 棕色背景
    44 蓝色背景
    45 品红背景
    46 孔雀蓝背景
    47 白色背景
    48 不知道什么东西
    49 设置默认背景色
    50-89 没用
    90-109 又是设置前景背景的，比之前的颜色浅
    \033[nA 光标上移n行
    \033[nB 光标下移n行
    \033[nC 光标右移n行
    \033[nD 光标左移n行
    \033[y;xH设置光标位置
    \033[2J 清屏
    \033[K 清除从光标到行尾的内容
    \033[s 保存光标位置
    \033[u 恢复光标位置
    \033[?25l 隐藏光标
    \033[?25h 显示光标*/
    
void coutColor(const std::string& info,int color/*=WHITE*/){
    switch(color){
        case COLOR_RED:         printf("\033[49;31m%s \033[0m",info.c_str()); break;
        case COLOR_GREEN:       printf("\033[49;32m%s \033[0m",info.c_str()); break;
        case COLOR_YELLOW:      printf("\033[49;33m%s \033[0m",info.c_str()); break;
        case COLOR_BLUE:        printf("\033[49;34m%s \033[0m",info.c_str()); break;
        case COLOR_PINK:        printf("\033[49;35m%s \033[0m",info.c_str()); break;
        case COLOR_DARKGREEN:   printf("\033[49;36m%s \033[0m",info.c_str()); break;
        case COLOR_WHITE:       printf("\033[49;37m%s \033[0m",info.c_str()); break;
        default: printf("%s",info.c_str()); break;
    }
    printf("\n");
}

void coutError(const std::string& module,const std::string& info)
{
    printf("\033[40;31m[ERROR] \033[0m");
    std::cout<<module<<":"<<info<<std::endl;

}
void coutWarn(const std::string& module,const std::string& info)
{
    printf("\033[40;32m[WARN] \033[0m");
    std::cout<<module<<":"<<info<<std::endl;
}

#endif

