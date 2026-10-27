#!/usr/bin/env python3
"""`fit_gripper_width.py` 的自测：已知直线要解回来，没量的行要跳过。

拟合这件事最容易出的错不是算错，是**把一个错点当真**或者**悄悄拿不到的点补零**。
所以这里钉三件事：能解回来、点数不够时拒绝、空读数不许被当成 0 mm。
"""
import io
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import fit_gripper_width as fit_tool   # noqa: E402

FAILS = []


def ok(text):
    print('  [ok  ] {}'.format(text))


def fail(text, detail=''):
    FAILS.append(text)
    print('  [FAIL] {}{}'.format(text, '：' + detail if detail else ''))


def close(label, got, want, tol=1e-6):
    if abs(got - want) <= tol:
        ok('{}（{}）'.format(label, round(got, 4)))
    else:
        fail(label, '得到 {}，期望 {}'.format(got, want))


def truth(revs):
    """真值线：开口 = 13.48 - 11.12 * 圈数"""
    return 13.48 - 11.12 * revs


def check_recover():
    print('\n1. 三个实测点要解回原来的两个常数')
    points = [(0.10, truth(0.10)), (0.25, truth(0.25)), (0.50, truth(0.50))]
    fit = fit_tool.fit_width(points)
    close('截距 = 13.48', fit['intercept'], 13.48, 1e-9)
    close('斜率 = -11.12', fit['slope'], -11.12, 1e-9)
    close('残差', fit['rms'], 0.0, 1e-9)

    print('\n2. 有噪声时要报告 RMS，而不是假装完美')
    noisy = [truth(r) + d for r, d in zip((0.0, 0.25, 0.5, 1.0), (0.3, -0.2, 0.1, -0.4))]
    fit = fit_tool.fit_width(list(zip((0.0, 0.25, 0.5, 1.0), noisy)))
    close('截距仍接近 13.48', fit['intercept'], 13.48, 0.45)
    close('斜率仍接近 -11.12', fit['slope'], -11.12, 0.6)
    if 0.0 < fit['rms'] < 0.5:
        ok('RMS {:.2f} mm 在合理范围'.format(fit['rms']))
    else:
        fail('RMS 不合理', fit['rms'])


def check_refusals():
    print('\n3. 点数不够就拒绝，不给假装可信的数')
    for label, points in (('一个点', [(0.1, 12.0)]),
                          ('圈数全一样', [(0.2, 12.0), (0.2, 11.5)]),
                          ('空表', [])):
        try:
            fit_tool.fit_width(points)
        except fit_tool.NotEnoughPoints:
            ok('{} 被拒绝'.format(label))
        else:
            fail('{} 居然解出来了'.format(label))


def check_parse():
    print('\n4. 手写 --inline 要认空格和分号')
    got = fit_tool.parse_inline(' 0.10,12.5 ; 0.25,10.5 ;0.50,8.0 ')
    if got == [(0.10, 12.5), (0.25, 10.5), (0.50, 8.0)]:
        ok('解析出 3 个点')
    else:
        fail('解析结果不对', str(got))
    try:
        fit_tool.parse_inline('0.10')
    except ValueError:
        ok('缺一列要报错')
    else:
        fail('缺一列没报错')


def check_csv():
    print('\n5. CSV：表头、注释、空行、没量的行都要跳过，不许补 0')
    text = ('# gripper_sweep.py 生成\n'
            'target_revs,readback_revs,width_mm,finished\n'
            '0.1000,0.1002,12.50,1\n'
            '\n'
            '0.2500,0.2501,,1\n'
            '0.5000,0.5000,8.00,1\n'
            '# 下面是坏的\n'
            'oops,0.7500,7.00,0\n')
    fd, path = tempfile.mkstemp(suffix='.csv')
    os.close(fd)
    try:
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(text)
        points = fit_tool.read_csv(path)
    finally:
        os.remove(path)
    if points == [(0.10, 12.50), (0.50, 8.00)]:
        ok('只剩两个有读数的点，没量的那行没被当成 0')
    else:
        fail('CSV 解析结果不对', str(points))


def check_report():
    print('\n6. report 要把两个常量印成能直接抄的样子')
    fit = fit_tool.fit_width([(0.0, 13.4), (1.0, 2.3)])
    buf = io.StringIO()
    fit_tool.report(fit, stream=buf)
    out = buf.getvalue()
    for text in ('REV_AT_ZERO_WIDTH_MM = 13.400', 'MM_PER_REV = 11.100'):
        if text in out:
            ok('印出了 {}'.format(text))
        else:
            fail('没印出 {}'.format(text), out)


def main():
    check_recover()
    check_refusals()
    check_parse()
    check_csv()
    check_report()
    print('\nSUMMARY')
    if FAILS:
        for text in FAILS:
            print('  [FAIL] {}'.format(text))
        print('RESULT: {} 处不对'.format(len(FAILS)))
        return 1
    print('  [ ok ] 拟合能解回已知直线，点数不够和空读数都按预期处理')
    print('RESULT: the width fit is trustworthy on known data')
    return 0


if __name__ == '__main__':
    sys.exit(main())
