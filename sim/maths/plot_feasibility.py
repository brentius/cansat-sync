"""GPL-3.0-only. Standard ReportLab charts; optional sharp SVG rasterization.

Run after run_feasibility.py. NumPy/Pillow/ReportLab are bundled artifact tools,
not new numerical or flight dependencies. Final charts also remain vector SVG.
"""
import csv
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from reportlab.graphics.charts.lineplots import LinePlot
from reportlab.graphics.charts.barcharts import VerticalBarChart
from reportlab.graphics import renderSVG
from reportlab.graphics.shapes import Drawing, String, Rect
from reportlab.lib import colors

OUT=Path(__file__).parent/'results'/'attachment_audit'
BLUE=colors.HexColor('#1761a0'); ORANGE=colors.HexColor('#d67524'); RED=colors.HexColor('#ae3434')


def chart_title(d,title,subtitle):
    d.add(Rect(0,0,d.width,d.height,fillColor=colors.white,strokeColor=None))
    d.add(String(65,335,title,fontName='Helvetica-Bold',fontSize=16,fillColor=colors.HexColor('#172b42')))
    d.add(String(65,315,subtitle,fontName='Helvetica',fontSize=10,fillColor=colors.HexColor('#4f5d6b')))


def line_chart(name,title,subtitle,data,labels,xlabel,ylabel,xlimits,ylimits):
    d=Drawing(740,365); chart_title(d,title,subtitle)
    chart=LinePlot(); chart.x=75; chart.y=70; chart.width=590; chart.height=220; chart.data=data
    chart.joinedLines=True; chart.xValueAxis.valueMin,chart.xValueAxis.valueMax=xlimits
    chart.yValueAxis.valueMin,chart.yValueAxis.valueMax=ylimits
    chart.xValueAxis.labels.fontSize=9; chart.yValueAxis.labels.fontSize=9
    chart.xValueAxis.visibleGrid=True; chart.yValueAxis.visibleGrid=True
    chart.xValueAxis.gridStrokeColor=chart.yValueAxis.gridStrokeColor=colors.HexColor('#e4e8ed')
    palette=[BLUE,ORANGE,RED,colors.black]
    for i,label in enumerate(labels):
        chart.lines[i].strokeColor=palette[i]; chart.lines[i].strokeWidth=2
        d.add(String(75+i*150,30,label,fontSize=10,fillColor=palette[i]))
    d.add(chart)
    d.add(String(335,50,xlabel,fontSize=10,textAnchor='middle'))
    d.add(String(10,294,ylabel,fontSize=10))
    renderSVG.drawToFile(d,str(OUT/name))


def main():
    report=json.loads((OUT/'summary.json').read_text())
    baseline=json.loads((OUT/'baseline'/'verification.json').read_text())
    image=report['images']['scenarios']; names=['flat','photometric','relief_2m','relief_5m','spin_blur_1ms']
    d=Drawing(740,365); chart_title(d,'Terrain relief dominates this rendered example','SYNTHETIC | 24 poses per scenario | periodic texture: zero accepted fixes')
    bars=VerticalBarChart(); bars.x=75; bars.y=70; bars.width=590; bars.height=220
    bars.data=[[image[n]['radial_rmse_m'] for n in names]]
    bars.categoryAxis.categoryNames=['Flat','Lighting','Relief 2 m','Relief 5 m','Spin blur']
    bars.valueAxis.valueMin=0; bars.valueAxis.valueMax=6; bars.valueAxis.valueStep=1
    bars.bars[0].fillColor=BLUE; bars.valueAxis.visibleGrid=True; bars.valueAxis.gridStrokeColor=colors.HexColor('#e4e8ed')
    d.add(bars); d.add(String(10,294,'Radial RMS (m)',fontSize=10))
    d.add(String(75,30,'Local matching with 640 x 400 images; these errors exclude map datum uncertainty.',fontSize=10))
    renderSVG.drawToFile(d,str(OUT/'terrain_error.svg'))
    curves=[]
    for scenario in ('matched','correlated_vision_noise','correlated_vision_noise_corrected'):
        curves.append(sorted([(x['height_m'],100*x['coverage95']) for x in baseline['scenarios'][scenario]['TRN']['landing']]))
    line_chart('landing_coverage.svg','Landing covariance needs the sensor-error model',
               'SYNTHETIC | 2,000 independent descents per scenario | nominal containment: 95%',
               curves,['Matched','AR1 ignored','AR1 tracked'],'Height remaining (m)','95% coverage (%)',(0,300),(40,100))
    with np.load(OUT/'atmosphere_raw.npz') as raw:
        t=raw['swing_time_s']; mask=t<=5
        data=[list(zip(t[mask][::5],raw['swing_velocity_mps'][mask][::5])),
              list(zip(t[mask][::5],raw['swing_false_wind_mps'][mask][::5]))]
        line_chart('shared_swing.svg','Sensor agreement can hide common false wind',
                   'SYNTHETIC | 15 degree linear swing | 1.5 m line | perfect velocity/derivative measurements',
                   data,['Payload swing speed','False retrieved wind'],'Time (s)','Perturbation (m/s)',(0,5),(-3.5,3.5))
    with np.load(OUT/'example_rendered_frames.npz') as frames:
        panel=Image.new('RGB',(960,645),'#f5f7fa'); draw=ImageDraw.Draw(panel)
        font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',18)
        draw.text((20,12),'Retained synthetic camera scenes (one example per scenario)',fill='#172b42',font=font)
        for j,name in enumerate(image):
            x=(j%3)*320; y=50+(j//3)*295
            frame=Image.fromarray(frames[name]).resize((310,194))
            panel.paste(frame,(x+5,y)); draw.text((x+10,y+202),name,fill='#172b42',font=font)
            draw.text((x+10,y+230),'640 x 400 source pixels',fill='#4f5d6b',font=font)
        panel.save(OUT/'rendered_scenes.png')


if __name__=='__main__': main()
