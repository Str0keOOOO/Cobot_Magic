# Cobot Magic

Cobot Magic 的基础使用、ROS 环境和机械臂操作请参考
[Cobot_Magic 1.0 用户手册 · AgileX 知识库](https://agilexsupport.yuque.com/staff-hso6mo/toh64r/tcpvae9wrb5xnivn?singleDoc#%20%E3%80%8A%E9%99%84%E4%BB%B61-cobot_magic%E8%AF%A6%E7%BB%86%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E6%96%87%E6%A1%A3%E3%80%8B)。

## 构建 ROS 工作区

在仓库根目录运行以下命令，构建当前机械臂使用的 RealSense D435 与 Piper ROS 包：

```bash
cd ~/Cobot_Magic
bash tools/build_cobot_magic.sh
```

## TiPToP 连接

本仓库额外提供 `tiptop_client/`：它运行在机械臂上位机上，连接现有 ROS、机械臂和右腕相机，并将控制与相机服务提供给 TiPToP 推理服务器。

如需配置或启动 TiPToP 客户端，请参考 [tiptop_client/README.md](tiptop_client/README.md)。
