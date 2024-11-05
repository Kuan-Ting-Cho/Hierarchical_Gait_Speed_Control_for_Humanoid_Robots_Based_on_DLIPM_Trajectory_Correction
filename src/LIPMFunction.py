import numpy as np

def StepSize2StrideLength(S): # StepSize:此步前腳與上一步前腳差值 StrideLength:左右腳每步的長度
    """ Output length of stride from input stepsize.
    """
    length = np.zeros(len(S)) 
    length[0] = S[0]
    if np.all(S == 0):  # a是啥?
        a = np.zeros(len(S))
    else:
        a = 0.005 * np.ones(len(S))
    firstLegLength = np.array([length[0]])
    secondLegLength = np.array([0])
    for i in range(1, len(S)):
        length[i] = S[i] + S[i - 1]  #左右腳每步的長度
        if (i % 2 == 1):
            firstLegLength = np.append(firstLegLength, [0])
            secondLegLength = np.append(secondLegLength, [length[i]])
        else:
            firstLegLength = np.append(firstLegLength, [length[i]])
            secondLegLength = np.append(secondLegLength, [0])

    return firstLegLength, secondLegLength, length, a


def StrideLength2ZMPAmplitude(length): #X的大小為A
    amplitude = np.zeros(len(length))
    amplitude[1] = length[0]
    for i in range(2, len(length)):
        amplitude[i] = length[i - 1] + amplitude[i - 2]
    return amplitude


def ZMPAmplitudeSign(amplitude, a, forward): #X的方向
    if (forward):
        return amplitude, a
    else:
        return -amplitude, -a


def ModifiableXOSGRampDSPRampSSP(amplitude, a, k, T, dt, wn_T): #完整CoM X軌跡、速度、加速度
    D1 = np.zeros(len(amplitude))
    d2 = np.zeros(len(amplitude))
    for i in range(len(amplitude) - 1):
        d2[i] = (amplitude[i + 1] - amplitude[i]) / 2
        D1[i + 1] = d2[i]

    x = np.array([])
    v = np.array([])
    p = np.array([])
    for i in range(len(amplitude)):
        bias = 0
        [xn, vn, pn] = XOSGZMPBC2RampDSPRampSSP(amplitude[i], a[i], D1[i],
                                                d2[i], bias, k, T, dt, wn_T)
        x = np.append(x, xn)
        v = np.append(v, vn)
        p = np.append(p, pn)
    return x, v, p


def XOSGZMPBC2RampDSPRampSSP(amplitude, a, D1, d2, bias, k, T, dt, wn_T):#一個step的x
    w = np.mean(wn_T)
    if k > 0:  # k = virtual DSP scale 
        t1 = k * T #公式2.22的參數
        t2 = (1 - k) * T
        l1 = amplitude - D1
        k1 = (D1 - a) / k / T 
        l2 = amplitude - a - 2 * a * t1 / (1 - 2 * k) / T
        k2 = 2 * a / (1 - 2 * k) / T 
        l3 = amplitude + a - (d2 - a) * t2 / k / T
        k3 = (d2 - a) / k / T 
        [_, X1S, _, _] = StePzMP2CoM(T, w, l1, 0)  #公式2.23的各項
        [_, X1R, _, _] = RamPzMP2CoM(T, w, k1, 0) 
        [_, X2S, _, _] = StePzMP2CoM(T, w, l2 - l1, t1)
        [_, X2R, _, _] = RamPzMP2CoM(T, w, k2 - k1, t1)
        [_, X3S, _, _] = StePzMP2CoM(T, w, l3 - l2, t2)
        [_, X3R, _, _] = RamPzMP2CoM(T, w, k3 - k2, t2)
        X1 = X1S + X1R 
        X2 = X2S + X2R
        X3 = X3S + X3R
    else:
        L1 = amplitude - a
        K1 = 2 * a / T
        [_, X1S, _, _] = StePzMP2CoM(T, w, L1, 0) 
        [_, X1R, _, _] = RamPzMP2CoM(T, w, K1, 0)
        X1 = X1S + X1R
        X2 = 0
        X3 = 0

    C = np.cosh(w * T)
    S = np.sinh(w * T)
    x0 = amplitude - D1 
    xf = amplitude + d2
    x_v0 = (xf - x0 * C - X1 - X2 - X3) * w / S #公式2.25

    w = wn_T
    X = []
    V = []
    A = []
    for i in range(1, int(T / dt) + 1):
        t = i * dt
        if k > 0:
            [_, X1S, V1S, A1S] = StePzMP2CoM(t, w, l1, 0)
            [_, X1R, V1R, A1R] = RamPzMP2CoM(t, w, k1, 0)
            [_, X2S, V2S, A2S] = StePzMP2CoM(t, w, l2 - l1, t1)
            [_, X2R, V2R, A2R] = RamPzMP2CoM(t, w, k2 - k1, t1)
            [_, X3S, V3S, A3S] = StePzMP2CoM(t, w, l3 - l2, t2)
            [_, X3R, V3R, A3R] = RamPzMP2CoM(t, w, k3 - k2, t2)
            x = x0*np.cosh(w*t) + x_v0/w*np.sinh(w*t) + X1S + X1R + \
                                                  X2S + X2R + \
                                                  X3S + X3R # 公式2.4 CoM X軌跡
            v = x0*w*np.sinh(w*t) + x_v0*np.cosh(w*t) + V1S + V1R + \
                                                        V2S + V2R + \
                                                        V3S + V3R # 公式2.5 CoM X速度
            a = x0*(w**2)*np.cosh(w*t) + x_v0*w*np.sinh(w*t) + A1S + A1R + \
                                                            A2S + A2R + \
                                                            A3S + A3R
        else:
            [_, X1S, V1S, A1S] = StePzMP2CoM(t, w, l1, 0)
            [_, X1R, V1R, A1R] = RamPzMP2CoM(t, w, k1, 0)
            x = x0 * np.cosh(w * t) + x_v0 / w * np.sinh(w * t) + X1S + X1R
            v = x0 * w * np.sinh(w * t) + x_v0 * np.cosh(w * t) + V1S + V1R
            a = x0 * (w**2) * np.cosh(w * t) + x_v0 * w * np.sinh(
                w * t) + A1S + A1R

        X.append(x)
        V.append(v)
        A.append(a)

    p = X - A / (w**2)
    p = bias + np.array(p)
    x = bias + np.array(X)
    v = np.array(V)
    return x, v, p


def StePzMP2CoM(t, w, As, Ts):
    p = As * np.array(t - Ts)
    x = As * (1 - np.cosh(w * (t - Ts))) * np.heaviside(t - Ts, 0.5)
    v = -As * w * np.sinh(w * (t - Ts)) * np.heaviside(t - Ts, 0.5)
    a = -As * w**2 * np.cosh(w * (t - Ts)) * np.heaviside(t - Ts, 0.5)
    return p, x, v, a


def RamPzMP2CoM(t, w, Ar, Tr):
    p = Ar * t * np.heaviside(t - Tr, 0.5)
    x = Ar * (t - Tr * np.cosh(w * (t - Tr)) -
              1 / w * np.sinh(w * (t - Tr))) * np.heaviside(t - Tr, 0.5)
    v = Ar * (1 - Tr * w * np.sinh(w * (t - Tr)) -
              np.cosh(w * (t - Tr))) * np.heaviside(t - Tr, 0.5)
    a = Ar * (-Tr * (w**2) * np.cosh(w * (t - Tr)) -
              w * np.sinh(w * (t - Tr))) * np.heaviside(t - Tr, 0.5)
    return p, x, v, a


def ModifiableYOSGRampDSPSinSSP(B, b, k, T, dt, wn_T, finalZero):#完整CoM Y軌跡、速度、加速度
    N = len(B)
    # print(N)
    D1 = np.zeros(N)
    d2 = np.zeros(N)
    if (~finalZero):
        d2[N - 1] = (B[N - 1] + B[N - 2]) / 2
    for i in range(N - 1):
        d2[i] = (B[i + 1] + B[i]) / 2
        D1[i + 1] = d2[i]

    y = np.array([])
    v = np.array([])
    p = np.array([])
    for i in range(N):
        bias = 0

        [yn, vn, pn] = YOSCGZMPBC2RampDSPSinSSP(B[i], b[i], D1[i], d2[i], bias,
                                                k, T, dt, wn_T)
        y = np.append(y, yn)
        v = np.append(v, vn)
        p = np.append(p, pn)
    return y, v, p


def YOSCGZMPBC2RampDSPSinSSP(B, b, y0, yf, bias, k, T, dt, wn_T): #一個step的y
    w = np.mean(wn_T)
    if k > 0:
        t1 = k * T
        t2 = (1 - k) * T
        l1 = y0
        k1 = (B - y0) / k / T
        l2 = B
        m2 = b
        l3 = B - (yf - B) * t2 / k / T
        k3 = (yf - B) / k / T
        alpha = np.pi / (1 - 2 * k) / T
        T_d = alpha * t1
        [_, Y1S, _, _] = StePzMP2CoM(T, w, l1, 0)
        [_, Y1R, _, _] = RamPzMP2CoM(T, w, k1, 0)
        [_, Y2S, _, _] = StePzMP2CoM(T, w, l2 - l1, t1)
        [_, Y2R, _, _] = RamPzMP2CoM(T, w, -k1, t1)
        [_, Y3S, _, _] = StePzMP2CoM(T, w, l3 - l2, t2)
        [_, Y3R, _, _] = RamPzMP2CoM(T, w, k3, t2)
        [_, Y2Sin, _, _] = SinTdZMP2CoM(T, w, m2, alpha, T_d, t1)
        [_, Y3Sin, _, _] = SinTdZMP2CoM(T, w, -m2, alpha, T_d, t2)
        Y1 = Y1S + Y1R
        Y2 = Y2S + Y2R + Y2Sin
        Y3 = Y3S + Y3R + Y3Sin
        C = np.cosh(w * T)
        S = np.sinh(w * T)
        y_v0 = (yf - y0 * C - Y1 - Y2 - Y3) * w / S
    else:
        b = 0
        k1 = 2 * b / T
        l1 = (B - b)
        C = np.cosh(w * T)
        S = np.sinh(w * T)
        y_v0 = (yf - y0 * C - k1 * (T - S / w) - l1 * (1 - C)) * w / S

    w = wn_T
    Y = []
    V = []
    A = []
    for i in range(1, int(T / dt) + 1):
        t = i * dt
        if k > 0:
            [_, Y1S, V1S, A1S] = StePzMP2CoM(t, w, l1, 0)
            [_, Y1R, V1R, A1R] = RamPzMP2CoM(t, w, k1, 0)
            [_, Y2S, V2S, A2S] = StePzMP2CoM(t, w, l2 - l1, t1)
            [_, Y2R, V2R, A2R] = RamPzMP2CoM(t, w, -k1, t1)
            [_, Y2Sin, V2Sin, A2Sin] = SinTdZMP2CoM(t, w, m2, alpha, T_d, t1)
            [_, Y3S, V3S, A3S] = StePzMP2CoM(t, w, l3 - l2, t2)
            [_, Y3R, V3R, A3R] = RamPzMP2CoM(t, w, k3, t2)
            [_, Y3Sin, V3Sin, A3Sin] = SinTdZMP2CoM(t, w, -m2, alpha, T_d, t2)
            y = y0*np.cosh(w*t) + y_v0/w*np.sinh(w*t) + Y1S + Y1R + \
                                                        Y2S + Y2R + Y2Sin + \
                                                        Y3S + Y3R + Y3Sin  # 公式2.4
            v = y0*w*np.sinh(w*t) + y_v0*np.cosh(w*t) + V1S + V1R + \
                                                        V2S + V2R + V2Sin + \
                                                        V3S + V3R + V3Sin  # 公式2.5
            a = y0*(w**2)*np.cosh(w*t) + y_v0*w*np.sinh(w*t) + A1S + A1R + \
                                                                A2S + A2R + A2Sin + \
                                                                A3S + A3R + A3Sin
        else:
            y = y0 * np.cosh(w * t) + y_v0 / w * np.sinh(w * t) + k1 * (
                t - 1. / w * np.sinh(w * t)) + l1 * (1 - np.cosh(w * t))
            v = y0 * w * np.sinh(w * t) + y_v0 * np.cosh(
                w * t) + k1 * (1 - np.cosh(w * t)) - l1 * w * np.sinh(w * t)
            a = y0 * (w**2) * np.cosh(w * t) + y_v0 * w * np.sinh(
                w * t) - k1 * w * np.sinh(w * t) - l1 * (w**2) * np.cosh(w * t)

        Y.append(y)
        V.append(v)
        A.append(a)

    q = Y - A / (w**2)
    y = bias + np.array(Y)
    q = bias + np.array(q)
    v = np.array(V)
    return y, v, q


def SinTdZMP2CoM(t, w, A_beta, alpha, T_d, T_beta):
    [Ps, Xs, Vs, As] = SinZMP2CoM(t, w, A_beta, alpha, T_beta)
    [Pc, Xc, Vc, Ac] = CosZMP2CoM(t, w, A_beta, alpha, T_beta)
    # print(Xs)
    p = Ps * np.cos(T_d) - Pc * np.sin(T_d)
    x = Xs * np.cos(T_d) - Xc * np.sin(T_d)
    v = Vs * np.cos(T_d) - Vc * np.sin(T_d)
    a = As * np.cos(T_d) - Ac * np.sin(T_d)
    return p, x, v, a


def SinZMP2CoM(t, wn, A_beta, alpha, T_beta):
    p = A_beta * np.sin(alpha * t) * np.heaviside(t - T_beta, 0.5)
    phi = A_beta * np.sin(alpha * T_beta)
    psi = alpha * A_beta * np.cos(alpha * T_beta)
    x = 1 / (alpha**2 + wn**2) * (
        phi * (wn**2) * np.cos(alpha * (t - T_beta)) + psi / alpha *
        (wn**2) * np.sin(alpha * (t - T_beta)) - phi *
        (wn**2) * np.cosh(wn * (t - T_beta)) -
        psi * wn * np.sinh(wn * (t - T_beta))) * np.heaviside(t - T_beta, 0.5)
    v = 1 / (alpha**2 + wn**2) * (
        -alpha * phi * (wn**2) * np.sin(alpha * (t - T_beta)) + psi *
        (wn**2) * np.cos(alpha * (t - T_beta)) - phi *
        (wn**3) * np.sinh(wn * (t - T_beta)) - psi *
        (wn**2) * np.cosh(wn * (t - T_beta))) * np.heaviside(t - T_beta, 0.5)
    a = 1 / (alpha**2 + wn**2) * (
        -phi * (alpha**22) * (wn**2) * np.cos(alpha *
                                              (t - T_beta)) - psi * alpha *
        (wn**2) * np.sin(alpha * (t - T_beta)) - phi *
        (wn**4) * np.cosh(wn * (t - T_beta)) - psi *
        (wn**3) * np.sinh(wn * (t - T_beta))) * np.heaviside(t - T_beta, 0.5)

    return p, x, v, a


def CosZMP2CoM(t, wn, A_beta, alpha, T_beta):
    p = A_beta * np.cos(alpha * t) * np.heaviside(t - T_beta, 0.5)
    phi = A_beta * np.cos(alpha * T_beta)
    psi = -alpha * A_beta * np.sin(alpha * T_beta)
    x = 1 / (alpha**2 + wn**2) * (
        phi * (wn**2) * np.cos(alpha * (t - T_beta)) + psi / alpha *
        (wn**2) * np.sin(alpha * (t - T_beta)) - phi *
        (wn**2) * np.cosh(wn * (t - T_beta)) -
        psi * wn * np.sinh(wn * (t - T_beta))) * np.heaviside(t - T_beta, 0.5)
    v = 1 / (alpha**2 + wn**2) * (
        -alpha * phi * (wn**2) * np.sin(alpha * (t - T_beta)) + psi *
        (wn**2) * np.cos(alpha * (t - T_beta)) - phi *
        (wn**3) * np.sinh(wn * (t - T_beta)) - psi *
        (wn**2) * np.cosh(wn * (t - T_beta))) * np.heaviside(t - T_beta, 0.5)
    a = 1 / (alpha**2 + wn**2) * (
        -phi * (alpha**2) * (wn**2) * np.cos(alpha *
                                             (t - T_beta)) - psi * alpha *
        (wn**2) * np.sin(alpha * (t - T_beta)) - phi *
        (wn**4) * np.cosh(wn * (t - T_beta)) - psi *
        (wn**3) * np.sinh(wn * (t - T_beta))) * np.heaviside(t - T_beta, 0.5)
    return p, x, v, a


def flipsign(Amp, len):
    B = np.linspace(1, len, len, endpoint=True)
    A = Amp * (B % 2) - Amp * abs(1 - B % 2)
    return A


def CalKLeanAngle(kDSP, len):
    A = np.array([kDSP / 2, 0.5, 1 - (kDSP / 2)])
    a = np.array([A])
    for _ in range(1, len):
        a = np.append(a, [A], axis=0)
    return a


def CompletedRGeneration(LeanAngleInit, kLeanAngle, LeanAngleArray, T, dt):#完整的R
    kSize = kLeanAngle.shape[0]
    R = np.array([])
    for i in range(kSize):
        # R = np.append(
        #     R,
        #     OneStepRGeneration(LeanAngleInit, kLeanAngle[i%2, :],
        #                        LeanAngleArray[i%2, :], T, dt))
        R = np.append(
            R,
            OneStepRGeneration(LeanAngleInit, kLeanAngle[i, :],
                               LeanAngleArray[i, :], T, dt))
    # print(R)
    return R


def OneStepRGeneration(LeanAngleInit, kLeanAngle, LeanAngleArray, T, dt): #一個step的R
    t = np.linspace(dt, T, int(T / dt), endpoint=True)
    R = np.zeros(len(t))
    t_k = np.insert(kLeanAngle, 0, 0)
    # print(t_k)
    t_k = np.append(t_k, [1])
    t_k = T * t_k
    # print(t_k)
    for i in range(4):
        dA = LeanAngleArray[i + 1] - LeanAngleArray[i]
        dT = t_k[i + 1] - t_k[i]
        if dT != 0:
            theta = (t - t_k[i]) * 2 * np.pi / dT
            K = dA / 2 / np.pi * (theta - np.sin(theta))
            R = R + (LeanAngleArray[i] + K) * (np.heaviside(
                t - t_k[i], 0.5) - np.heaviside(t - t_k[i + 1], 0.5))
    R[len(R) - 1] = LeanAngleArray[len(LeanAngleArray) - 1]
    return R + LeanAngleInit


def ModifiableFootGeneration(Amp, T, dt, tDSP): #p.20
    N = Amp.shape[1]
    length_t = int(np.floor(N * T / dt))
    footX = np.zeros((1, length_t))
    footY = np.zeros((1, length_t))
    footZ = np.zeros((1, length_t))
    # print(footX.shape)
    t1 = tDSP / 2
    t2 = T - tDSP / 2
    tSSP = T - tDSP
    xBias = 0
    yBias = 0
    i = 0
    tIndex = 1
    nIndex = 0
    tReal = 0

    t = np.linspace(dt, N * T, int(N * T / dt), endpoint=True)
    for t_temp in t:
        t = tIndex * dt
        xAmp = Amp[0, nIndex]
        yAmp = Amp[1, nIndex]
        zAmp = Amp[2, nIndex]
        footX[0, i] = xBias + xAmp / 2 / np.pi * (
            2 * np.pi *
            (t - t1) / tSSP - np.sin(2 * np.pi * (t - t1) / tSSP)) * (
                np.heaviside(t - t1 - dt, 0.5) - np.heaviside(t - t2, 0.5))
        footX[0, i] = footX[0, i] + xAmp * np.heaviside(t - t2, 0.5) #2.39
        footY[0, i] = yBias + yAmp / 2 / np.pi * (
            2 * np.pi *
            (t - t1) / tSSP - np.sin(2 * np.pi * (t - t1) / tSSP)) * (
                np.heaviside(t - t1 - dt, 0.5) - np.heaviside(t - t2, 0.5))
        footY[0, i] = footY[0, i] + yAmp * np.heaviside(t - t2, 0.5) #2.40
        footZ[0, i] = zAmp / 2 * (1 - np.cos(2 * np.pi * (t - t1) / tSSP)) * (
            np.heaviside(t - t1 - dt, 0.5) - np.heaviside(t - t2, 0.5)) #2.41
        i = i + 1
        tIndex = tIndex + 1
        tReal = tReal + dt
        if tReal > (nIndex + 1) * T:
            xBias = xBias + Amp[0, nIndex]
            yBias = yBias + Amp[1, nIndex]
            tIndex = 1
            nIndex = nIndex + 1
            if nIndex == N:
                break

    Foot = np.append([footX], [footY], axis=0)
    Foot = np.append(Foot, [footZ], axis=0)
    return Foot


def OutputMotion(PR, PL, hip1, hip2, leanAngleR, leanAngleL, yawAngleR,
                 yawAngleL, pedalRollAngleR, pedalRollAngleL, legLinkLength,d2,turn, rightFirst, T, dt):
    N = PR.shape[1]
    thetaR = []
    thetaL = []
    RR=[]
    LR=[]
    for i in range(N):
        RR.append(np.array([[np.cos(leanAngleR[i]), 0,
                       np.sin(leanAngleR[i])], [0, 1, 0],
                      [-np.sin(leanAngleR[i]), 0,
                       np.cos(leanAngleR[i])]]))
        P = PR[:, i]
        r_Leg = InvK(P, RR[i], legLinkLength,d2)
        LR.append(np.array([[np.cos(leanAngleL[i]), 0,
                       np.sin(leanAngleL[i])], [0, 1, 0],
                      [-np.sin(leanAngleL[i]), 0,
                       np.cos(leanAngleL[i])]]))
        P = PL[:, i]
        l_Leg = InvK(P, LR[i], legLinkLength,d2)
        r_Leg[5] += pedalRollAngleR[i]
        l_Leg[5] += pedalRollAngleL[i]

        if turn:
            r_Leg[0] = yawAngleR[i]
            l_Leg[0] = yawAngleL[i]

        currentStep = np.floor(i * dt / T)
        if (currentStep % 2 == 0):
            if rightFirst:
                l_Leg[1] = hip1 * l_Leg[1]
            else:
                r_Leg[1] = hip1 * r_Leg[1]
        else:
            if rightFirst:
                r_Leg[1] = hip2 * r_Leg[1]
            else:
                l_Leg[1] = hip2 * l_Leg[1]

        thetaR.append(r_Leg)
        thetaL.append(l_Leg)

    offSetR = np.array(thetaR[0]).reshape((1, 6))
    offSetL = np.array(thetaL[0]).reshape((1, 6))
    outputR = thetaR - offSetR
    outputL = thetaL - offSetL
    outputData = np.append(outputR, outputL, axis=1)
    outputData = np.round(outputData, 3)

    return outputData,RR,LR

# def InvK(P, R, L, d2):

    q = np.zeros(6)

    [Px, Py, Pz] = P

    [L2, L3, L4, L5, L6] = L

    [r11, r12, r13] = R[0]
    [r21, r22, r23] = R[1]
    [r31, r32, r33] = R[2]

    q[1] = np.arctan(-(Py + L6 * r23) / (L6 * r33 + Pz))
    q[5] = np.arcsin(-r23 * np.cos(q[1]) - r33 * np.sin(q[1]))

    H04x = Px + r13 * (L6 + L5 * np.cos(q[5])) + L5 * r12 * np.sin(q[5])
    H04y = Py + r23 * (L6 + L5 * np.cos(q[5])) + L5 * r22 * np.sin(q[5])
    H04z = Pz + r33 * (L6 + L5 * np.cos(q[5])) + L5 * r32 * np.sin(q[5])

    H02x = d2
    H02y = L2 * np.sin(q[1])
    H02z = -L2 * np.cos(q[1])

    H04 = [H04x, H04y, H04z]
    H02 = [H02x, H02y, H02z]
    L24 = np.sqrt((H04[0] - H02[0])**2 + (H04[1] - H02[1])**2 +
                  (H04[2] - H02[2])**2)
    theta = (L24**2 - L3**2 - L4**2) / (2 * L3 * L4)

    if theta > 1:
        theta = 1
    elif theta < -1:
        theta = -1
    else:
        theta = theta
    q[3] = np.arccos(theta)

    alpha = Py * np.sin(q[1]) - Pz * np.cos(q[1]) - (L6 + L5 * np.cos(
        q[5])) * (r33 * np.cos(q[1]) - r23 * np.sin(q[1])) - L2 - L5 * np.sin(
            q[5]) * (r32 * np.cos(q[1]) - r22 * np.sin(q[1]))
    beta = d2 - Px - r13 * (L6 + L5 * np.cos(q[5])) - L5 * r12 * np.sin(q[5])
    gamma = L3 + L4 * np.cos(q[3])
    phi = L3 * gamma + L4**2 * np.sin(q[3])**2 + L4 * np.cos(q[3]) * gamma

    test_sin = (beta * gamma - alpha * L4 * np.sin(q[3])) / phi
    if (test_sin > 1 or test_sin < -1):
        print("theta 3 error !")

    q[2] = np.arcsin((beta * gamma - alpha * L4 * np.sin(q[3])) / phi)

    A = np.cos(q[5]) * (r33 * np.cos(q[1]) - r23 * np.sin(q[1])) + np.sin(
        q[5]) * (r32 * np.cos(q[1]) - r22 * np.sin(q[1]))
    B = r13 * np.cos(q[5]) + r12 * np.sin(q[5])

    q[4] = np.arcsin(B * np.cos(q[2] + q[3]) - A * np.sin(q[2] + q[3]))

    return q

def InvK(P,R,L,d2): #Roli
    [Px,Py,Pz] = P
    [r11, r12, r13] = R[0]
    [r21, r22, r23] = R[1]
    [r31, r32, r33] = R[2]
    D1, D2, D3= -0.064 ,0.006 ,-0.00725
    [L2, L3, L4, L5, L6] = L
    #theta2
    a = Pz + L6 * r33 - D1
    b = Py + L6 * r23
    r = (a**2 + b**2)**(1/2)
    t2 = -np.arctan2(b, a) + np.arctan2(D3/r, -(1 - (D3/r))**(1/2))
    while t2 >np.pi:
        t2-=2*np.pi
    while t2 <-np.pi:
        t2+=2*np.pi
        
    #theta6
    t6 = np.arcsin(-r23 * np.cos(t2) - r33 * np.sin(t2))

    #theta4
    H04x = Px + r13 * (L6 + L5 * np.cos(t6)) + L5 * r12 * np.sin(t6)
    H04y = Py + r23 * (L6 + L5 * np.cos(t6)) + L5 * r22 * np.sin(t6)
    H04z = Pz + r33 * (L6 + L5 * np.cos(t6)) + L5 * r32 * np.sin(t6)

    H02x = D2
    H02y = L2 * np.sin(t2)
    H02z = D1 - L2 * np.cos(t2)

    H04 = [H04x, H04y, H04z] #Motor5 position-
    H02 = [H02x, H02y, H02z] #Motor3 position

    L35 = np.sqrt((H04[0] - H02[0])**2 + (H04[1] - H02[1])**2 +  #L35
                    (H04[2] - H02[2])**2)
    L35 = L35 * np.sin(np.arccos(0.00725/L35))
    l3 = 0.35795
    l4 = 0.36642
    theta = (l3**2 + l4**2 - L35**2) / (2 * l3 * l4) #theta4

    if theta > 1:
        theta = 1
    elif theta < -1:
        theta = -1
    else:
        theta = theta
    t4 = np.pi - np.arccos(theta) - np.arccos(0.366/0.36642)

    c4 = np.cos(t4 + np.pi*2.74/180)
    s4 = (1-c4**2)**(1/2)

    #theta3
    alpha = D1 * np.cos(t2) + Py * np.sin(t2) - Pz * np.cos(t2) - (L6 + L5 * np.cos(
        t6)) * (r33 * np.cos(t2) - r23 * np.sin(t2)) - L2 - L5 * np.sin(
            t6) * (r32 * np.cos(t2) - r22 * np.sin(t2))  #B
    beta = D2 - Px - r13 * (L6 + L5 * np.cos(t6)) - L5 * r12 * np.sin(t6) #A
    gamma = L3 + L4 * c4
    phi = L3 * gamma + L4**2 * s4**2 + L4 * c4 * gamma #sin(theta3) 的分母

    test_sin = (beta * gamma - alpha * L4 * s4) / phi 
    if (test_sin > 1 or test_sin < -1):
        # print("theta 3 error !")
        t3 = 0.0
    else:
        t3 = np.arcsin((beta * gamma - alpha * L4 * s4) / phi ) #theta3
    #theta5
    A = np.cos(t6) * (r33 * np.cos(t2) - r23 * np.sin(t2)) + np.sin(
        t6) * (r32 * np.cos(t2) - r22 * np.sin(t2)) #C
    B = r13 * np.cos(t6) + r12 * np.sin(t6) #D

    t5 = np.arcsin(B * np.cos(t3 + t4 + np.pi*2.74/180) - A * np.sin(t3 + t4 + np.pi*2.74/180)) + np.pi*2.74/180 #theta5
    return np.array([0,t2,t3,t4,t5,t6])