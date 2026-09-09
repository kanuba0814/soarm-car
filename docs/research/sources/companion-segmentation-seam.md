# 评估专用可见实例真值：已核实官方接缝

官方版本匹配文档 https://gazebosim.org/api/sensors/6/segmentationcamera_igngazebo.html 明确支持 Fortress起的 segmentation sensor。不是尚待发明mesh raycaster。实际本机运行仍待验证。

SDF: `<sensor name="eval_segmentation" type="segmentation"><topic>eval_segmentation</topic><camera><segmentation_type>semantic</segmentation_type><horizontal_fov>1.047</horizontal_fov><image><width>640</width><height>480</height></image><clip><near>0.1</near><far>8</far></clip></camera><update_rate>15</update_rate></sensor>`；放在当前camera_link相同外参，生成运行时模型副本，不改native model。为每个物理实例给唯一1..255 label即可用semantic标签当实例ID（当前87模型足够），避免依赖panoptic动态instance count。label插件可附在world include: `<plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label"><label>10</label></plugin>`。未标物体为背景，必须验证未标墙仍正常遮挡，不出现透视标签。

输出 `/eval_segmentation/labels_map` 与 `/eval_segmentation/colored_map`，native ignition::msgs::Image，须检查运行时格式、RGB/BGR字节顺序。官方两种例子索引顺序不同，不能盲照抄。先单个独立已知色标签图形验证字节解码、相同像素网格和遮挡，再用真实家具轮廓叠图至少3视角验证。保持深度/RGB/标签源时间同步。需要核查给include附插件是否正确作用嵌套visual。

GT标签和源实例身份只能给评估端，策略进程不能看到标签话题、标签图、GT文件、source world/asset挂载或标注名字。图像文件名中用opaque observation ID，别塞答案。采集broker可分别导出public RGB-D/self-pose与private GT。共享视图用于开发识别可行性；正式测试布局冻结后不调参。该接缝可使无人值守独立评分可行，不等于已经实现或证明导航。
