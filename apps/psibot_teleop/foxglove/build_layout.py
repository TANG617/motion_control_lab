#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["foxglove-sdk==0.26.0"]
# ///
"""Generate the Cortex debug layout; consumes existing telemetry only."""
import argparse
import json
from pathlib import Path
import foxglove.layouts as fl

PREFIX = '/psi_cortex/'
URDF_URL = 'http://127.0.0.1:8766/Psi_R1_visual_collision.urdf'
COLORS = ['#56B4E9', '#E69F00', '#009E73', '#CC79A7', '#F0E442', '#D55E00', '#AAAAFF']
STAGES = {'goal': '#009E73', 'reference': '#E69F00', 'command': '#CC79A7', 'actual': '#56B4E9'}


def split(direction, *items):
    return fl.SplitContainer(direction=direction, items=[fl.SplitItem(content=p, proportion=w) for w, p in items])


def tabs(items):
    return fl.TabContainer(tabs=[fl.TabItem(title=title, content=p) for title, p in items])


def note(title, text):
    return fl.MarkdownPanel(title=title, config=fl.MarkdownConfig(markdown=text, font_size=13))


def raw(title, topic):
    return fl.RawMessagesPanel(title=title, config=fl.RawMessagesConfig(topic_path=PREFIX + topic))


def series(label, path, color, dashed=False):
    # Foxglove "Log time": Cortex sets the WebSocket message timestamp to the
    # sample timestamp. Its custom int64-sec Time struct does not render here.
    return fl.PlotSeries(value=PREFIX + path, label=label, color=color, enabled=True,
        line_style='dashed' if dashed else 'solid', line_size=1.5, show_line=True,
        timestamp_method='receiveTime')


def plot(title, unit, paths):
    return fl.PlotPanel(title=title, config=fl.PlotConfig(paths=paths, y_axis_label=unit,
        is_synced=True, show_legend=True, legend_display='floating', show_plot_values_in_legend=False,
        time_window_mode='sliding', following_view_width=15, x_axis_val='timestamp'))


def states(title, paths):
    return fl.StateTransitionsPanel(title=title, config=fl.StateTransitionsConfig(
        is_synced=True, time_window_mode='sliding', x_axis_range=15,
        paths=[fl.StateTransitionsSeries(value=PREFIX + path, label=label, enabled=True,
            timestamp_method='receiveTime')
            for label, path in paths]))


def tracking_errors():
    paths=[]
    for i, side in enumerate(['L', 'R']):
        base=f'telemetry/tracking.arms[{i}]'
        paths.append(series(side + ' command → actual', base + '{command_fk_valid==true}{actual_fk_valid==true}.command_actual_position_error_m.@mul(1000)', COLORS[i]))
        paths.append(series(side + ' reference → actual (WBC)', base + '{reference_valid==true}{actual_fk_valid==true}.reference_actual_position_error_m.@mul(1000)', COLORS[i], True))
    return plot('末端位置误差 | command/reference → actual', 'mm', paths)


def scene(url):
    layers={name: fl.BaseRendererUrdfLayerSettings(visible=True, layer_id='foxglove.Urdf',
        instance_id=name, label=label, source_type='url', url=url, control_mode='jointStates',
        joint_states_topic=PREFIX+'joints/'+name, display_mode='visual', opacity=opacity,
        show_outlines=name=='command') for name,label,opacity in
        [('actual','Actual · solid',1.),('command','Command · transparent',.3)]}
    layers['grid']=fl.BaseRendererGridLayerSettings(visible=True, layer_id='foxglove.Grid',
        instance_id='grid', label='Base plane', size=4, divisions=20)
    topics={PREFIX+'cartesian/trajectories': fl.BaseRendererSceneUpdateTopicSettings(visible=True)}
    for stage in ['goal','reference','fk/command','fk/actual']:
        for side in ['left','right']:
            topics[PREFIX+'cartesian/'+stage+'/'+side]=fl.BaseRendererPoseTopicSettings(visible=True,
                axis_scale=.08 if stage=='goal' else .055)
    return fl.ThreeDeePanel(title='R1 | actual 实体 / command 半透明', config=fl.ThreeDeeConfig(
        layers=layers, topics=topics, follow_tf='base_link', fixed_frame='base_link',
        scene=fl.BaseRendererSceneSettings(transforms=fl.BaseRendererTransforms(show_label=False, axis_size=.12)),
        camera_state=fl.ThreeDeeCameraState(distance=3.4, perspective=True, phi=75,
            theta_offset=-45, target_offset=(.2,0,.9), near=.01, far=100)))


def cartesian(side, i):
    panels=[]
    for axis, j in zip('XYZ', range(3)):
        p=[]
        for stage in ['goal','reference']:
            p.append(series(stage + ' (WBC)', f'telemetry/wbc.arms[{i}]'+'{'+stage+'_valid==true}.'+stage+f'[{j}]', STAGES[stage]))
        for stage in ['command','actual']:
            p.append(series(stage, f'telemetry/tracking.arms[{i}]'+'{'+stage+'_fk_valid==true}.'+stage+f'_fk[{j}]', STAGES[stage]))
        panels.append((1, plot(f'{side} TCP · {axis}', 'm', p)))
    return split('column', *panels)


def joint_page(label, indices, names):
    panels=[]
    for field, unit in [('position','rad'),('velocity','rad/s'),('acceleration','rad/s²')]:
        paths=[]
        for color,(i,name) in zip(COLORS,zip(indices,names)):
            for stage in ['actual','command']:
                path='telemetry/control{'+stage+'.'+field+'_valid==true}.'+stage+'.'+field+f'[{i}]'
                paths.append(series(name+' '+stage, path, color, stage=='command'))
        panels.append((1,plot(label+' · '+field+' | actual 实线 / command 虚线',unit,paths)))
    return split('column', *panels)


def build(url):
    control_states=states('服务端会话与执行状态', [('session','telemetry/control.session'),
        ('outcome','telemetry/control.outcome'),('estop','telemetry/control.estop'),
        ('actual read OK','telemetry/control.actual_read_ok'),('write attempted','telemetry/control.write_attempted'),
        ('all writes OK','telemetry/control.all_writes_ok')])
    intro=note('操作与数据边界', '''**连接：** `ws://127.0.0.1:8765`。键盘控制仍在 teleop TUI。

单臂首次运行：`c → 左/右臂 → CartesianPosition`，再 `c → Enable`，最后 `Space`。
Space 只开始/暂停提交；暂停不代表制动。

**Part：** command/actual 和关节曲线有效。现有服务端不发布单臂 SDK 目标；goal/reference 仅用于 WBC。

**Mock：** actual 是理想位置回显，不代表硬件、动力学或力矩跟踪。

WBC 页只在服务端 WBC 会话中产生数据；空白/旧数据不表示求解成功。曲线采用采样时间。''')
    overview=split('row',(3,scene(url)),(2,split('column',(3,tracking_errors()),(2,control_states),(2,intro))))
    cart=split('column',(4,split('row',(1,cartesian('Left',0)),(1,cartesian('Right',1)))),
        (1,states('参考有效性 | Part 会话没有 WBC reference', [('L reference valid','telemetry/tracking.arms[0].reference_valid'),('R reference valid','telemetry/tracking.arms[1].reference_valid')])))
    joint=tabs([('Left arm',joint_page('Left',range(6,13),[f'L{i}' for i in range(1,8)])),
        ('Right arm',joint_page('Right',range(13,20),[f'R{i}' for i in range(1,8)])),
        ('Waist',joint_page('Waist',range(2,6),['torso yaw','torso pitch','knee','ankle'])),
        ('Head',joint_page('Head',range(2),['head yaw','head pitch']))])
    wbc=split('row',
        (1,split('column',
            (2,states('WBC / Yellow acceptance',[(name,'telemetry/wbc.'+name) for name in ['attempted','accepted','coupling_active','latest_yellow_accepted']]+[('Yellow accepted','telemetry/yellow.accepted')])),
            (2,plot('Red / Yellow 求解耗时','ms',[series('Red','telemetry/wbc{attempted==true}.elapsed_ms',COLORS[0]),series('Yellow','telemetry/yellow.elapsed_ms',COLORS[1])])),
            (1,plot('Red hard violation','solver native',[series('hard violation','telemetry/wbc{attempted==true}.hard_violation',COLORS[5])])))),
        (1,split('column',
            (1,plot('WBC task scales','ratio',[series(f'scale[{i}]',f'telemetry/wbc{{attempted==true}}.scales[{i}]',COLORS[i]) for i in range(2)])),
            (2,tabs([('Red / passes',raw('WBC attempts / passes','telemetry/wbc')),('Yellow',raw('Yellow attempts','telemetry/yellow'))])),
            (1,note('WBC 解释','Red 使用 `attempted=true` 过滤数值。accepted 是该次求解接受状态，不能视为目标运动完成。\n\n硬约束违反量、HQP pass 和 Yellow 原始字段来自 Cortex 可视化通道；它们不经过 Psi SDK。')))))
    timing=plot('控制周期 elapsed / deadline lateness','ms',[series('elapsed','telemetry/control.elapsed_ns.@mul(0.000001)',COLORS[0]),series('lateness','telemetry/control.deadline_lateness_ns.@mul(0.000001)',COLORS[5])])
    transport=plot('Telemetry queues | depths / drops','samples',[series(f'{["Part", "WBC", "Yellow"][i]} queue',f'telemetry/transport.queue_depth[{i}]',COLORS[i]) for i in range(3)]+[series(f'{["Part", "WBC", "Yellow"][i]} dropped',f'telemetry/transport.dropped[{i}]',COLORS[i],True) for i in range(3)])
    events=fl.LogPanel(title='Cortex events',config=fl.LogConfig(topic_to_render=PREFIX+'events',show_time=True,show_level=True,show_name=True))
    runtime=split('row',(1,split('column',(1,timing),(1,transport),(1,events))),
        (1,tabs([('Run / joint order',raw('配置与控制关节顺序','run/info')),('Control',raw('Control snapshot','telemetry/control')),('Transport',raw('队列与完整性','telemetry/transport'))])))
    return fl.Layout(content=tabs([('01 概览',overview),('02 末端跟踪',cart),('03 关节 PVA',joint),('04 WBC / Yellow',wbc),('05 运行与事件',runtime)]))


def file_import(api):
    """Convert typed layout containers to Foxglove desktop file-import mosaic."""
    configs={}
    def panel(kind,config):
        key=f'{kind}!psibot{len(configs)+1}'
        configs[key]=config
        return key
    def walk(node):
        if node['type']=='panel':
            config=dict(node['config'])
            if 'title' in node: config['foxglovePanelTitle']=node['title']
            return panel({'ThreeDee':'3D','Log':'RosOut'}.get(node['panelType'],node['panelType']),config)
        if node['type']=='tabs':
            children=[{'title':t['title'],'layout':walk(t['content'])} for t in node['tabs']]
            return panel('Tab',{'activeTabIdx':node['selectedTabIndex'],'tabs':children})
        def rest(items):
            if len(items)==1:return walk(items[0]['content'])
            return {'direction':node['direction'],'first':walk(items[0]['content']),
                'second':rest(items[1:]),'splitPercentage':100*items[0].get('proportion',1)/sum(i.get('proportion',1) for i in items)}
        return rest(node['items'])
    root=walk(api['content'])
    return {'configById':configs,'globalVariables':{},'userNodes':{},'playbackConfig':{'speed':1},'layout':root}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf-url',default=URDF_URL)
    parser.add_argument('--output-dir',type=Path,default=Path(__file__).resolve().parent)
    args=parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    api=json.loads(build(args.urdf_url).to_json())
    for name,content in [('psibot_teleop.layout.json',file_import(api)),('psibot_teleop.layout.api.json',api)]:
        path=args.output_dir/name
        path.write_text(json.dumps(content,ensure_ascii=False,indent=2)+'\n')
        print(path)


if __name__=='__main__':main()
