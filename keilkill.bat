@echo off
echo 正在清理Keil编译垃圾文件...
del *.bak /s
del *.ddk /s
del *.edk /s
del *.lst /s
del *.lnp /s
del *.mpf /s
del *.mpj /s
del *.obj /s
del *.omf /s
::del *.opt /s  ::注释掉！不要删除工程调试配置
del *.plg /s
del *.rpt /s
del *.tmp /s
del *.__i /s
del *.crf /s
del *.o /s
del *.d /s
del *.axf /s
del *.tra /s
del *.dep /s
del JLinkLog.txt /s
del *.iex /s
del *.htm /s
del *.map /s
del *.hex /s
del *.uvguix.* /s
echo 清理完成！
pause
