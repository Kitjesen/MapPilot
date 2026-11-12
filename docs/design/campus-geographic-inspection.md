# 园区地理点与巡检计划

本轮入口在正式地图页左上操作坞的「城市地图」。建图／导航页面都可打开，不触发机器人
Product 模式切换。弹窗内两个页签：

- **地理点位**：地图选点、机器人 GNSS 位置、手动坐标、文件导入、排序、删除及导出。
- **巡检计划**：复用现有日历与调度 API，绑定已经保存的本地地图巡检路线及版本。

计划页的「路线来源」可展开选择地图，只读取该地图路线，不切换机器人当前地图。
移除常驻地图选择及报错区域；服务离线时路线显示「路线暂不可用」，展开来源可看原因。
本机 `4326` 预览未启动原生巡检服务和调度线程，路线接口返回 503，不能提交现场巡检任务。
日历为单层月视图，单次计划只选一天，重复计划才展开起止日期；执行记录按需展开。

地理点以草稿保存在当前站点的浏览器存储中，不自动成为机器人路线，不与其他设备同步。
导出文件可在另一台浏览器导入。刷新、关闭弹窗后保留已经添加的点；尚未添加的输入不保存。
地理点草稿与日历中的可执行路线分开，页面明确标记尚未对齐本地地图。

## 导入

最大 1 MB、500 点。接受标准 RFC 7946 GeoJSON `FeatureCollection`，其中所有 geometry
必须是 `Point`，坐标顺序为 `[longitude, latitude, optional_altitude]`，使用 WGS84。
当前任务只保留水平位置，文件中的高度不用于导航。拒绝 legacy `crs` 字段和非点几何。
坐标校验失败时整批拒绝，保留原有草稿。

LingTu JSON 用于保留混合来源的原始坐标：

```json
{
  "version": 1,
  "points": [
    {"name": "入口", "longitude": 116.391, "latitude": 39.907, "datum": "WGS84"},
    {"name": "高德选点", "longitude": 116.397, "latitude": 39.908, "datum": "GCJ02"}
  ]
}
```

上面坐标仅为格式示例，不是现场可执行目标。导入追加到现有草稿，顺序可调整。

## 高德底图部署

使用现有 Gateway 同源代理，无新增依赖。优先使用运行时配置，不需要重新构建前端：

1. 登录 https://console.amap.com/ ，在应用管理中创建应用和 **Web 端（JS API）** Key，
   获取配套安全密钥。配置该 Key 的访问域名限制时，应覆盖实际部署的页面来源。
2. Windows 编辑 `%LOCALAPPDATA%/LingTu/amap.json`（本机已创建空模板）；Linux 部署为
   `/etc/lingtu/amap.json`，文件只允许服务账号读取。示例见 `config/amap.json.example`。
3. 填写 `key` 与 `security_code` 两个字段。配置文件不要提交仓库；不要发送到聊天。
4. 重新打开城市地图。首次补齐配置无需重启服务／构建前端；更换已加载的 Key 后刷新页面。

可用 `LINGTU_AMAP_CONFIG` 指定文件位置，或通过环境变量 `LINGTU_AMAP_KEY` 与
`LINGTU_AMAP_SECURITY_CODE` 成对注入。环境变量配置需按部署方式重启服务。

`GET /api/v1/geography/map-config` 只返回状态、公共 Key、代理路径，永不返回安全密钥。
状态 `configured` 仅表示两项配置齐全，不证明高德授权、域名白名单或配额有效。
`/_AMapService/*` 仅代理高德固定主机的 v3/v4 GET 请求，安全密钥由服务器添加，不跟随
上游重定向，不转发浏览器 Cookie，也不把上游异常 URL 返回客户端。

已有独立代理的部署仍支持构建时配置：

```dotenv
VITE_AMAP_KEY=<部署方高德 JS API 公共 Key>
VITE_AMAP_SERVICE_HOST=<部署方安全代理地址，以 /_AMapService 结尾>
```

`VITE_*` 会公开到浏览器，只能配置公开 Key 与代理地址。**安全密钥放在服务器代理，不能
放进 VITE 环境变量或源码。** 此种构建配置需重新构建。无配置时仍可输入、导入、
排序、导出点位，明确显示底图未配置。

高德点击产生 GCJ-02 点；GPS 的 WGS84 只通过官方 `AMap.convertFrom(..., 'gps', ...)`
转换为显示坐标，原始草稿不变。未实现逆向坐标近似作为运动定位。

官方参考：
- https://developer.amap.com/api/javascript-api-v2/guide/transform/convertfrom
- https://lbs.amap.com/api/javascript-api-v2/guide/abc/jscode

## GNSS 只读 API

`GET /api/v1/geography/location`，`Cache-Control: no-store`。

读取已有定位进程原子写入的 `/dev/shm/lingtu/gnss_status.json`；可由
`LINGTU_GNSS_STATUS_FILE` 指定。Gateway 不打开串口、不推断当前地图对齐状态。

状态为 `ready / unavailable / no_fix / stale / invalid`。只有有效 fix、合法经纬度、
且接收时间在 3 秒内时返回 location；未来超过 1 秒的时间戳拒绝。无有效测量误差时
`accuracy_m=null`，不以 HDOP 换算虚构精度。Web 的「机器人位置」按需读取，不读电脑定位。

## 真实执行还缺的接线

园区地理点的执行应经过：

1. 原始坐标与坐标系，明确保留点来源。
2. 当前本地地图的标定：地图 ID、内容版本、WGS84 原点、ENU 到 map 的旋转／平移、
   标定误差和有效范围。单独配置 GNSS 原点不能证明已经完成这一步。
3. Native 3D 导航的目标候选、支撑／净空／覆盖检查，以及路径检查。
4. 页面同时显示用户选点和实际候选，展示偏移或拒绝原因；确认后保存本地巡检路线。
5. 通过现有任务准入／控制权和调度接口执行，由 native navigation → driver → Brainstem
   负责最终运动。

本轮未增加地理点运动派发接口，也没有把经纬度直接传入本地 x/y。高德底图、GNSS fix、
浏览器成功保存草稿均不能代替本地地图标定和通行验证。
