particles = [(X, Y, theta, float(1) / N)] * N


def MotionPrediction(D, alpha):
    sd_x = 3
    sd_y = 3
    sd_a_1 = 0.02
    sd_a_2 = 0.05
    if (D == 0):
        particles = [(x, y, t + alpha + random.gauss(0, sd_a_2), w) for (x, y, t, w) in particles]
    else:
        particles = [(x + (D + random.gauss(0, sd_x))*math.cos(t), y + (D + random.gauss(0, sd_y))*math.sin(t), t + random.gauss(0, sd_a_1), w) for (x, y, t, w) in particles]
    return

def calcLikelihood(x,y,z):
    #z is sonar reading
    m = heightMap[x][y]
    sd_sonar = 4
    k = 1
    likelihood = math.exp(float(-((z-m)**2)) / (2*(sd_sonar**2)))
    return likelihood + K
    
def MeasurmentUpdate(z):
        particles = [(x, y, t, calcLikelihood(x, y, z)) for (x, y, t, w) in particles]
        
def normalize():
    sumP = sum([w for (_,_,_,w) in particles])
    particles = [(x, y, t, w/sumP) for (x, y, t, w) in particles]
    return

def resample():
    parray = [0] * N
    #create cumulative array
    for i in range(0, N):
        parray[i] = sum([w for (_,_,_,w) in particles.data[0:i + 1]])
    newp = [0] * N
    for i in range(0, N):
        w = random.uniform(0, 1)
        for j in range(0, N):
            if (parray[j] > w):
                newp[i] = particles.data[j]
                break
    particles.data = newp
    #Set all w to 1/N
    particles.data = [(x, y, t, float(1) / particles.n) for (x, y, t, _) in particles.data]
    return
