# Flutter Android 客户端开发计划

## 🎯 目标

创建一个 Android 应用，实时监控和控制机器人导航系统。

**优势**: Android 原生支持 gRPC，无需代理！直接连接 `192.168.66.190:50051`

---

## 📱 功能需求确认

### 核心功能（必须实现）

#### 1. 实时监控 📊
- [x] **连接管理**
  - 机器人 IP 输入（默认 192.168.66.190）
  - 连接状态指示
  - 自动重连机制
  
- [x] **位姿显示**
  - 位置（X, Y, Z）- 米
  - 姿态（Roll, Pitch, Yaw）- 度
  - 实时更新（10 Hz）
  
- [x] **速度监控**
  - 线速度（m/s）
  - 角速度（rad/s）
  - 速度仪表盘可视化
  
- [x] **系统状态**
  - 机器人 ID 和名称
  - 软件/固件版本
  - 运行时长

#### 2. 数据流监控 📡
- [ ] **话题频率统计** ⭐
  - `/Odometry` 频率
  - `/terrain_map` 频率  
  - `/path` 频率
  - LiDAR 点云频率
  
- [ ] **TF 状态检查** ⭐
  - `map → odom` 是否可用
  - `odom → body` 是否可用
  - 延迟统计

#### 3. 系统资源 💻
- [ ] **资源监控** ⭐
  - CPU 使用率
  - 内存使用率
  - 温度监控
  - 磁盘空间

#### 4. 可视化 📈
- [ ] **2D 轨迹图** ⭐
  - 实时位置轨迹
  - 历史路径显示
  - 缩放和平移

- [ ] **数据图表**
  - 速度曲线图
  - 频率统计图
  - CPU/内存趋势

#### 5. 控制功能 🎮
- [ ] **基础控制** ⭐
  - 紧急停止按钮
  - 启动/暂停导航
  
- [ ] **高级控制**（需要租约）
  - 发送目标点
  - 速度调整
  - 模式切换

#### 6. 事件和日志 📝
- [ ] **事件流** ⭐
  - 实时事件显示
  - 告警/错误提示
  - 事件历史记录

---

## 📋 推荐功能优先级

### 第一阶段（当前已完成）✅
- [x] 连接管理
- [x] 位姿显示
- [x] 速度监控
- [x] 基本系统状态

### 第二阶段（立即实施）⭐
```
优先级顺序：
1. 话题频率统计       （重要性: ⭐⭐⭐⭐⭐）
2. 系统资源监控       （重要性: ⭐⭐⭐⭐）
3. 事件流显示         （重要性: ⭐⭐⭐⭐）
4. TF 状态检查        （重要性: ⭐⭐⭐）
5. 紧急停止按钮       （重要性: ⭐⭐⭐⭐⭐）
```

### 第三阶段（后续扩展）
- 2D 轨迹可视化
- 数据图表
- 高级控制功能
- 任务管理

---

## 🛠️ 实施步骤

### 阶段 1：Android 环境配置

#### 1.1 添加 Android 平台支持

```bash
cd /home/sunrise/data/SLAM/navigation/client/flutter_monitor

# 添加 Android 平台（如果还没有）
flutter create --platforms=android .
```

#### 1.2 配置 Android 权限

编辑 `android/app/src/main/AndroidManifest.xml`：

```xml
<manifest xmlns:android="http://schemas.android.com/apk/res/android">
    
    <!-- 网络权限（必需） -->
    <uses-permission android:name="android.permission.INTERNET"/>
    <uses-permission android:name="android.permission.ACCESS_NETWORK_STATE"/>
    
    <!-- 可选：保持唤醒 -->
    <uses-permission android:name="android.permission.WAKE_LOCK"/>
    
    <application
        android:label="Robot Monitor"
        android:name="${applicationName}"
        android:icon="@mipmap/ic_launcher"
        android:usesCleartextTraffic="true">  <!-- 允许 HTTP 连接 -->
        
        <!-- ... -->
    </application>
</manifest>
```

#### 1.3 配置应用信息

编辑 `android/app/build.gradle`：

```gradle
android {
    namespace "com.example.robot_monitor"
    compileSdkVersion 34

    defaultConfig {
        applicationId "com.example.robot_monitor"
        minSdkVersion 24  // Android 7.0+
        targetSdkVersion 34
        versionCode 1
        versionName "1.0.0"
    }
}
```

---

### 阶段 2：实现第二阶段功能

#### 2.1 添加话题频率统计

在 `lib/screens/status_screen.dart` 中添加：

```dart
class _StatusScreenState extends State<StatusScreen> {
  // 频率统计变量
  Map<String, double> topicFrequencies = {
    'odom': 0.0,
    'terrain_map': 0.0,
    'path': 0.0,
    'lidar': 0.0,
  };

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: ListView(
        children: [
          // ... 现有的位姿、速度卡片
          
          // 新增：话题频率卡片
          Card(
            child: Column(
              children: [
                ListTile(
                  leading: Icon(Icons.speed, color: Colors.blue),
                  title: Text('话题频率统计'),
                ),
                _buildFrequencyRow('Odometry', topicFrequencies['odom']!),
                _buildFrequencyRow('Terrain Map', topicFrequencies['terrain_map']!),
                _buildFrequencyRow('Path', topicFrequencies['path']!),
                _buildFrequencyRow('LiDAR', topicFrequencies['lidar']!),
              ],
            ),
          ),
        ],
      ),
    );
  }
  
  Widget _buildFrequencyRow(String name, double freq) {
    Color color = freq > 5 ? Colors.green : freq > 1 ? Colors.orange : Colors.red;
    return ListTile(
      title: Text(name),
      trailing: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Text('${freq.toStringAsFixed(1)} Hz', 
               style: TextStyle(color: color, fontWeight: FontWeight.bold)),
          SizedBox(width: 8),
          Icon(Icons.circle, color: color, size: 12),
        ],
      ),
    );
  }
}
```

#### 2.2 添加系统资源监控

在 `robot_client.dart` 中订阅 SlowState：

```dart
Stream<SlowState> streamSlowState() {
  if (!_isConnected) {
    throw Exception('Not connected to robot');
  }

  final request = SlowStateRequest();
  return _telemetryClient.streamSlowState(request);
}
```

在 `status_screen.dart` 中显示：

```dart
Card(
  child: Column(
    children: [
      ListTile(
        leading: Icon(Icons.memory, color: Colors.orange),
        title: Text('系统资源'),
      ),
      _buildResourceRow('CPU', '${slowState.resources.cpuPercent}%'),
      _buildResourceRow('内存', '${slowState.resources.memoryPercent}%'),
      _buildResourceRow('温度', '${slowState.resources.temperature}°C'),
    ],
  ),
)
```

#### 2.3 添加紧急停止按钮

```dart
FloatingActionButton(
  onPressed: () async {
    // 发送停止指令
    await client.emergencyStop();
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(content: Text('紧急停止已触发！')),
    );
  },
  backgroundColor: Colors.red,
  child: Icon(Icons.stop, size: 32),
)
```

---

### 阶段 3：UI 优化（手机适配）

#### 3.1 响应式布局

```dart
class StatusScreen extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    final screenWidth = MediaQuery.of(context).size.width;
    final isTablet = screenWidth > 600;
    
    return Scaffold(
      body: isTablet
        ? Row(  // 平板横屏布局
            children: [
              Expanded(child: _buildLeftPanel()),
              Expanded(child: _buildRightPanel()),
            ],
          )
        : ListView(  // 手机竖屏布局
            children: [
              _buildLeftPanel(),
              _buildRightPanel(),
            ],
          ),
    );
  }
}
```

#### 3.2 深色模式支持

```dart
MaterialApp(
  theme: ThemeData.light(useMaterial3: true),
  darkTheme: ThemeData.dark(useMaterial3: true),
  themeMode: ThemeMode.system,  // 跟随系统
  home: ConnectionScreen(),
)
```

---

### 阶段 4：构建和测试

#### 4.1 构建 Debug APK

```bash
cd /home/sunrise/data/SLAM/navigation/client/flutter_monitor

# 构建 debug APK
flutter build apk --debug

# APK 位置
# build/app/outputs/flutter-apk/app-debug.apk
```

#### 4.2 构建 Release APK

```bash
# 生成签名密钥（首次）
keytool -genkey -v -keystore ~/robot-monitor.jks \
  -keyalg RSA -keysize 2048 -validity 10000 \
  -alias robot-monitor

# 配置 android/key.properties
storePassword=<密码>
keyPassword=<密码>
keyAlias=robot-monitor
storeFile=/home/sunrise/robot-monitor.jks

# 构建 release APK
flutter build apk --release
```

#### 4.3 安装到 Android 设备

```bash
# 通过 ADB 安装
adb install build/app/outputs/flutter-apk/app-release.apk

# 或者通过 Flutter 直接运行
flutter run -d <device-id>
```

---

## 📊 功能实现检查清单

### 当前已实现 ✅
- [x] 连接管理界面
- [x] 实时位姿显示
- [x] 速度监控
- [x] 基本系统信息
- [x] gRPC 客户端封装

### 第二阶段待实现 ⏳
- [ ] 话题频率统计（高优先级）
- [ ] 系统资源监控（高优先级）
- [ ] 紧急停止按钮（高优先级）
- [ ] 事件流显示
- [ ] TF 状态检查

### 第三阶段（可选）
- [ ] 2D 轨迹可视化
- [ ] 历史数据图表
- [ ] 高级控制功能
- [ ] 任务管理界面

---

## 🎨 UI 设计建议

### 主界面布局

```
┌─────────────────────────────────┐
│  Robot Monitor      [连接状态]   │
├─────────────────────────────────┤
│                                 │
│  📍 位姿                         │
│  X: 12.5m  Y: -3.2m  Z: 0.1m   │
│  Roll: 0°  Pitch: 2°  Yaw: 45° │
│                                 │
│  🚀 速度                         │
│  线速度: 0.5 m/s                │
│  角速度: 0.2 rad/s              │
│                                 │
│  📡 话题频率                     │
│  Odometry:     10.2 Hz  🟢     │
│  Terrain Map:   5.1 Hz  🟢     │
│  Path:          2.0 Hz  🟡     │
│  LiDAR:        10.0 Hz  🟢     │
│                                 │
│  💻 系统资源                     │
│  CPU:  45%  [████░░░░░░]       │
│  内存: 62%  [██████░░░░]       │
│  温度: 58°C                     │
│                                 │
├─────────────────────────────────┤
│           [🛑 紧急停止]           │
└─────────────────────────────────┘
```

---

## 📱 测试计划

### 单元测试
```bash
flutter test
```

### 集成测试
1. 连接测试（WiFi 环境）
2. 数据流测试（10Hz FastState）
3. UI 响应测试
4. 异常处理测试

### 性能测试
- 电池消耗
- 内存占用
- 网络流量

---

## 🚀 部署方式

### 方式 1：通过文件传输
```bash
# 构建 APK
flutter build apk --release

# 使用 adb 安装
adb install app-release.apk
```

### 方式 2：通过 GitHub Releases
上传 APK 到 GitHub Releases，用户可下载安装。

### 方式 3：内部分发
使用 Firebase App Distribution 或类似服务。

---

## 📝 下一步行动

### 立即开始（第二阶段功能）

1. **添加话题频率统计** (30分钟)
   - 修改 `status_screen.dart`
   - 添加频率显示组件

2. **添加系统资源监控** (20分钟)
   - 订阅 SlowState 流
   - 显示 CPU/内存/温度

3. **添加紧急停止按钮** (15分钟)
   - 添加 FloatingActionButton
   - 实现 emergencyStop 方法

4. **配置 Android 权限** (10分钟)
   - 修改 AndroidManifest.xml
   - 测试网络连接

5. **构建和测试 APK** (20分钟)
   - 构建 debug APK
   - 在真机上测试

**总计时间**: ~1.5 小时

---

## 💡 技术要点

### Android 优势
- ✅ 原生支持 gRPC（无需 Envoy）
- ✅ 性能好，延迟低
- ✅ 可以后台运行
- ✅ 支持通知推送

### 注意事项
- ⚠️ Android 9+ 默认禁止明文 HTTP，需配置 `usesCleartextTraffic`
- ⚠️ 需要 INTERNET 权限
- ⚠️ 考虑电池优化（降低更新频率）

---

## 📚 参考资料

- [Flutter Android 部署](https://docs.flutter.dev/deployment/android)
- [gRPC Dart 文档](https://grpc.io/docs/languages/dart/)
- [Android 权限配置](https://developer.android.com/guide/topics/permissions/overview)

---

## 🎯 最终目标

**一个功能完善、性能优异的 Android 机器人监控应用！**

✅ 实时监控  
✅ 系统状态  
✅ 紧急控制  
✅ 用户友好
