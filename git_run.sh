# # 初始化本地仓库：
git init
# # 查看代码状态
git status
# # 添加文件，"."会添加当前路径的所有文件，可以换成指定的文件
git add .
# # 提交说明
git commit -m 'firt add'
# # 连接远程仓库
git remote add origin https://github.com/zhujun3753/livox_hesai_calibration.git
# # 移动/重命名一个分支，即使目标已存在
git branch -M main
# # 将代码push到远程仓库（-u）第一次
git push -u origin main

# git status
# git add .
# git commit -m 'first add'
# git push origin main