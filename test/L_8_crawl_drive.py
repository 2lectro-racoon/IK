# crawl_drive_forward_simple.py
# ==========================================
# 4족보행 Crawl Gait 교육용 코드 (단순화 버전)
#
# W : 전진
# P : 기본자세 복귀
#
# 보행 순서
# BR → FR → BODYMOVE → BL → FL → BODYMOVE
#
# 특징
# - 원래 로직 유지
# - 함수 수 최소화
# - 강의용으로 읽기 쉽게 정리
# ==========================================

import sys
import time
import select
import termios
import tty
from dataclasses import dataclass

from A_quad_api import make_default_api
from A_ik_3dof_a0 import IKError


# ==========================================
# 기본 설정
# ==========================================

STAND_X = 60
STAND_Y = 120
STAND_Z = -50

LIFT_DZ = 60
STEP_FWD = 40

FB_SWING_SCALE = 2.0
BODYMOVE_FWD = 1.25

MOVE_DT = 0.02
PHASE_T = 0.2
BODYMOVE_T = 0.35
IDLE_HOLD = 0.35


FRONT_LEGS = {0,3}
BACK_LEGS = {1,2}


# ==========================================
# 키 명령 구조
# ==========================================

@dataclass
class Cmd:
    vx:int=0
    reset:bool=False


def key_to_cmd(k):

    k=k.lower()

    if k=="w":
        return Cmd(vx=1)

    if k=="p":
        return Cmd(reset=True)

    return Cmd()


# ==========================================
# 키보드 입력
# ==========================================

class KeyReader:

    def __init__(self):
        self.fd=sys.stdin.fileno()
        self.old=termios.tcgetattr(self.fd)

    def __enter__(self):
        tty.setcbreak(self.fd)
        return self

    def __exit__(self,a,b,c):
        termios.tcsetattr(self.fd,termios.TCSADRAIN,self.old)

    def read(self):

        r,_,_=select.select([sys.stdin],[],[],0.0)

        if not r:
            return None

        return sys.stdin.read(1)


# ==========================================
# Crawl Driver
# ==========================================

class CrawlDriver:

    def __init__(self):

        self.api=make_default_api()

        self.foot={
            0:(STAND_X,STAND_Y,STAND_Z),
            1:(STAND_X,STAND_Y,STAND_Z),
            2:(STAND_X,STAND_Y,STAND_Z),
            3:(STAND_X,STAND_Y,STAND_Z),
        }

        self.idx=0


    # ==========================================
    # 다리 이동 (보간 포함)
    # ==========================================

    def move_leg(self,leg,x2,y2,z2,t):

        x1,y1,z1=self.foot[leg]

        steps=max(1,int(t/MOVE_DT))

        for i in range(steps+1):

            u=i/steps

            x=x1+(x2-x1)*u
            y=y1+(y2-y1)*u
            z=z1+(z2-z1)*u

            try:
                self.api.set_leg_xyz(leg,x,y,z)
            except IKError:
                return False

            time.sleep(MOVE_DT)

        self.foot[leg]=(x2,y2,z2)

        return True


    # ==========================================
    # 기본 자세
    # ==========================================

    def go_stand(self):

        for i in range(4):
            self.move_leg(i,STAND_X,STAND_Y,STAND_Z,0)


    # ==========================================
    # 다리 Swing
    #
    # 1. Lift
    # 2. Forward
    # 3. Down
    # ==========================================

    def swing_leg(self,leg):

        x,y,z=self.foot[leg]

        # Lift
        if not self.move_leg(leg,x,y,z+LIFT_DZ,PHASE_T):
            return False

        # Forward
        dx=STEP_FWD*FB_SWING_SCALE

        if leg not in FRONT_LEGS:
            dx=-dx

        if not self.move_leg(leg,x+dx,y,z+LIFT_DZ,PHASE_T):
            return False

        # Down
        if not self.move_leg(leg,x+dx,y,STAND_Z,PHASE_T):
            return False

        return True


    # ==========================================
    # BODY MOVE (Push)
    #
    # 몸 전체를 앞으로 밀어주는 단계
    # ==========================================

    def body_push(self):

        dx=-(STEP_FWD*BODYMOVE_FWD)

        steps=max(1,int(BODYMOVE_T/MOVE_DT))

        start=self.foot.copy()

        for i in range(steps+1):

            u=i/steps

            for leg in range(4):

                x,y,z=start[leg]

                if leg in FRONT_LEGS:
                    nx=x+dx*u
                else:
                    nx=x-dx*u

                try:
                    self.api.set_leg_xyz(leg,nx,y,STAND_Z)
                except IKError:
                    return False

            time.sleep(MOVE_DT)

        for leg in range(4):

            x,y,_=start[leg]

            if leg in FRONT_LEGS:
                x+=dx
            else:
                x-=dx

            self.foot[leg]=(x,y,STAND_Z)

        return True


    # ==========================================
    # 전진 순서
    #
    # BR → FR → PUSH → BL → FL → PUSH
    # ==========================================

    def forward_step(self):

        seq=[1,0,-1,2,3,-1]

        item=seq[self.idx%len(seq)]

        self.idx+=1

        if item==-1:

            if not self.body_push():
                self.go_stand()

            return

        if not self.swing_leg(item):
            self.go_stand()


    # ==========================================
    # 메인 동작
    # ==========================================

    def crawl_step(self,cmd):

        if cmd.reset:
            print("기본 자세")
            self.go_stand()
            return

        if cmd.vx==1:
            self.forward_step()


# ==========================================
# main
# ==========================================

def main():

    drv=CrawlDriver()

    drv.api.leg_reset()

    time.sleep(4)

    print("W = 전진")
    print("P = 기본자세")

    drv.go_stand()

    cmd=Cmd()
    last_key=0

    with KeyReader() as kr:

        while True:

            k=kr.read()

            now=time.time()

            if k:

                cmd=key_to_cmd(k)
                last_key=now

            if now-last_key>IDLE_HOLD:
                cmd=Cmd()

            drv.crawl_step(cmd)


if __name__=="__main__":
    main()