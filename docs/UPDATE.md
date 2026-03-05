# Update
## ARM, FOOT 모터 조립, 초기 상태
- 본체와 4개 ARM Connector를 연결하는 모터는 혼의 방향이 윗쪽이다.
- 모터의 초기 각도는 90도로 설정하고 ARM는 본체의 대각선상에 위치하도록 조립했다.
- 각 모터의 이름은 A(front-left), B(front-right), C(back-left), D(back-right) 이다.
- ARM 모터는 모터 축이 움직이미로 반시계 방향 회전한다.
- ARM 모터가 모두 동일 방향으로 회전하므로 이것을 고려해서 coding해야한다.
- 각 ARM에에 연결된 FOOT Connector는 모터의 초기 각도를 120도로 설정하고 FOOT 본체,ARM과 평행하도록 조립했다.(수평)

 
## 2026.03.04
### Walk
- AA,BA,CA,DA만 움직이고 AF~DF는 움직이지 않는다. 
- AA~DA는 동시에 반시계 방향으로 45도- 시계방향 90도- AA 반시계 방향 90도 - CA 반시계 방향 90도 - CA 시계 방향 90도 - BA 반시계 방향 90도 - BA 시계 방향 90도 - DA 반시계 방향 90도 

