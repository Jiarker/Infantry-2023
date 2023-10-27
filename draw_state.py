import matplotlib.pyplot as plt

temp_speed = []
temp_time = []

correct_speed = []
correct_time = []

now_speed = []
now_time = []

num = 0
sum = []
delta_angle = []
delta_speed = []

with open("./src/Algorithm/configure/Detector/fittool/buff_state/14.txt", "r") as speed_txt:
    for line in speed_txt.readlines():
        curLine = line.strip().split(" ")
        floatLine = map(float,curLine) 
        floatLine = list(floatLine)

        temp_speed.append(floatLine[0])
        temp_time.append(floatLine[1])

        delta_angle.append(floatLine[2])
        delta_speed.append(floatLine[3])

        now_speed.append(floatLine[4])
        now_time.append(floatLine[5])

        for i in range(6, len(floatLine), 1):
            if i % 2 == 0 :
                correct_speed.append(floatLine[i])
            else :
                correct_time.append(floatLine[i])

        sum.append(num)
        num = num + 1


fig, ax = plt.subplots(nrows=2, ncols=1, sharex=False, sharey=False)

ax[0].set_xlim(correct_time[0],correct_time[-1])
ax[0].set_ylim(-3,3)
ax[0].scatter(correct_time, correct_speed, s=5)
ax[0].scatter(now_time, now_speed, s=5)
ax[0].set_title("correct_and_now_speed")

ax[1].set_xlim(temp_time[0],int(temp_time[-1]))
ax[1].set_ylim(-3,3)
ax[1].scatter(temp_time, temp_speed, s=5)
ax[1].set_title("temp_speed")

plt.show()

fig1, ax1 = plt.subplots(nrows=2, ncols=1, sharex=False, sharey=False)

ax1[0].set_xlim(0, num)
ax1[0].set_ylim(-0.2,1)
ax1[0].scatter(sum, delta_angle,s=5)
ax1[0].set_title("delta_angle")

ax1[1].set_xlim(0, num)
ax1[1].set_ylim(-0,1)
ax1[1].scatter(sum, delta_speed,s=5)
ax1[1].set_title("delta_speed")

plt.show()