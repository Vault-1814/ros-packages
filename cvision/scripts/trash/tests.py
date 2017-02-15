import cv2
import imutils
import scripts.cv_methods_factory.exmodules.cvgui as tb


def getBorderType(bt):
    """for opencv 3.2"""
    bts = (
        cv2.BORDER_CONSTANT,
        cv2.BORDER_REPLICATE,
        cv2.BORDER_REFLECT,
        cv2.BORDER_WRAP,
        cv2.BORDER_REFLECT_101,
        cv2.BORDER_TRANSPARENT,
        cv2.BORDER_REFLECT101,
        cv2.BORDER_DEFAULT,
        cv2.BORDER_ISOLATED)
    bts_name = (
        'BORDER_CONSTANT',
        'BORDER_REPLICATE',
        'BORDER_REFLECT',
        'BORDER_WRAP',
        'BORDER_REFLECT_101',
        'BORDER_TRANSPARENT',
        'BORDER_REFLECT101',
        'BORDER_DEFAULT',
        'BORDER_ISOLATED')
    #print(bts_name[bt], ' was selected!')
    return bts[bt]


def getFilterName(typeBlur):
    if typeBlur == 0:
        return 'bilateral filter'
    elif typeBlur == 1:
        return 'blur'
    elif typeBlur == 2:
        return 'box filter'
    elif typeBlur == 3:
        return 'build pyramid'
    elif typeBlur == 4:
        return 'erode'
    elif typeBlur == 5:
        return 'filter2D'
    elif typeBlur == 6:
        return 'GaussianBlur'
    elif typeBlur == 7:
        return 'medianBlur'
    elif typeBlur == 8:
        return 'Laplacian'

def getTrackbarsForFilter(typeBlur):
    if typeBlur == 0:
        fd = tb.Trackbar('DIAMETER_PX', (0, 10, -1))
        fsigC = tb.Trackbar('SIG_COLOR', (0, 255, 0))
        fsigS = tb.Trackbar('SIG_SPACE', (0, 255, 0))
        return fd, fsigC, fsigS
    elif typeBlur == 1:
        fksize = tb.Trackbar('KSIZE', (0, 100, 1))
        # twice because in trackbar module something bug if once, lol
        return fksize, fksize
    elif typeBlur == 2:
        fddepth = tb.Trackbar('DDEPTH', (0, 8, -1))
        fksize = tb.Trackbar('KSIZE', (0, 50, 1))
        fnormalize = tb.Trackbar('NORMOLIZE', (0, 1, 0))
        return fddepth, fksize, fnormalize
    elif typeBlur == 3:
        fshape = tb.Trackbar('SHAPE_TYPE', (0, 2, 0))
        fk = tb.Trackbar('KSIZE', (0, 50, 1))
        fits = tb.Trackbar('ITERS', (0, 50, 0))
        return fshape, fk, fits
    elif typeBlur == 4:
        fshape = tb.Trackbar('SHAPE_TYPE', (0, 2, 0))
        fk = tb.Trackbar('KSIZE', (0, 50, 1))
        fits = tb.Trackbar('ITERS', (0, 50, 0))
        return fshape, fk, fits
    elif typeBlur == 5:
        fshape = tb.Trackbar('SHAPE_TYPE', (0, 2, 0))
        fk = tb.Trackbar('KSIZE', (0, 50, 1))
        fddepth = tb.Trackbar('DDEPTH', (0, 50, 0))
        return fshape, fk, fddepth
    elif typeBlur == 6:
        fksize = tb.Trackbar('KSIZE', (0, 100, 1))
        fsx = tb.Trackbar('SIGMAX', (0, 300, 0))
        fsy = tb.Trackbar('SIGMAY', (0, 300, 0))
        return fksize, fsx, fsy
    elif typeBlur == 7:
        fksize = tb.Trackbar('KSIZE', (0, 100, 1))
        return fksize, fksize
    elif typeBlur == 8:
        fddepth = tb.Trackbar('DDEPTH', (0, 8, -1))
        fk = tb.Trackbar('KSIZE', (0, 7, 1))
        fsc = tb.Trackbar('SCALE', (0, 20, 1))
        fdt = tb.Trackbar('DELTA', (0, 300, 0))
        return fddepth, fk, fsc, fdt,

def filteringImage(image, typeBlur, tbs):
    if typeBlur == 0:
        """d=-1 for compute itself; d=5 for online; d=9 for offline"""
        d, sigC, sigS, btType = tbs.get_trackbar_values()
        bt = getBorderType(btType)
        if imutils.is_cv2():
            if bt == 5:
                bt = 4
        image = cv2.bilateralFilter(image, d, sigC, sigS, borderType=bt)
    elif typeBlur == 1:
        k, k, btType = tbs.get_trackbar_values()
        ksize = (k, k)
        bt = getBorderType(btType)
        image = cv2.blur(image, ksize, borderType=bt)
    elif typeBlur == 2:
        ddepth, k, n, btType = tbs.get_trackbar_values()
        ksize = (k, k)
        bt = getBorderType(btType)
        image = cv2.boxFilter(image, ddepth, ksize, normalize=n, borderType=bt)
    elif typeBlur == 3:
        shape, k, its, btType = tbs.get_trackbar_values()
        ksize = (k, k)
        bt = getBorderType(btType)
        if shape == 0:
            morph = cv2.MORPH_RECT
        elif shape == 1:
            morph = cv2.MORPH_CROSS
        else:
            morph = cv2.MORPH_ELLIPSE
        kernel = cv2.getStructuringElement(morph, ksize)
        image = cv2.dilate(image, kernel, iterations=its, borderType=bt)
    elif typeBlur == 4:
        shape, k, its, btType = tbs.get_trackbar_values()
        ksize = (k, k)
        bt = getBorderType(btType)
        if shape == 0:
            morph = cv2.MORPH_RECT
        elif shape == 1:
            morph = cv2.MORPH_CROSS
        else:
            morph = cv2.MORPH_ELLIPSE
        kernel = cv2.getStructuringElement(morph, ksize)
        image = cv2.erode(image, kernel, iterations=its, borderType=bt)
    elif typeBlur == 5:
        shape, k, ddepth, btType = tbs.get_trackbar_values()
        ksize = (k, k)
        bt = getBorderType(btType)
        if shape == 0:
            morph = cv2.MORPH_RECT
        elif shape == 1:
            morph = cv2.MORPH_CROSS
        else:
            morph = cv2.MORPH_ELLIPSE
        kernel = cv2.getStructuringElement(morph, ksize)
        image = cv2.filter2D(image, ddepth, kernel, delta=0)
    elif typeBlur == 6:
        k, sX, sY, btType = tbs.get_trackbar_values()
        if k % 2 == 1:
            ksize = (k, k)
        else:
            ksize = (k-1, k-1)
        bt = getBorderType(btType)
        image = cv2.GaussianBlur(image, ksize, sigmaX=sX, sigmaY=sY, borderType=bt)
    elif typeBlur == 7:
        k, k, btType = tbs.get_trackbar_values()
        if k % 2 == 0:
            k -= 1
        image = cv2.medianBlur(image, k)
    elif typeBlur == 8:
        ddepth, k, sc, dt, btType = tbs.get_trackbar_values()
        if k % 2 == 0:
            k -= 1
        ksize = cv2.getDerivKernels(2, 2, ksize=k)
        bt = getBorderType(btType)
        image = cv2.Laplacian(
            image, ddepth, ksize=k, scale=sc, delta=dt, borderType=bt)
    return image


fTypeBlur = tb.Trackbar('TYPE_BLUR', (0, 8, 8))
tbTB = tb.TrackbarWindow('SELECT_TYPE_BLUR_FILTER', fTypeBlur)
oldTypeBlur = -1

fBorderType = tb.Trackbar('BORDER_TYPE', (0, 8, 0))

#cap = cv2.VideoCapture(1)
while True:
    img = cv2.imread('/media/data/evo/py_projects/cv/raw_images/for_descriptor.jpg')
    #_, img = cap.read()
    if type(img[0, 0]) is not list:
        w, h, _ = img.shape
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        w, h = img.shape
    scale = 2
    img = cv2.resize(img, (h / scale, w / scale))

    typeBlur = tbTB.get_trackbar_value('TYPE_BLUR')
    if typeBlur != oldTypeBlur:
        if 'tbs' in locals():
            cv2.destroyWindow(tbs.get_win_name())
        tbs = tb.TrackbarWindow(getFilterName(typeBlur))
        tbs.add_filters(getTrackbarsForFilter(typeBlur))
        tbs.add_filter(fBorderType)
    oldTypeBlur = typeBlur
    try:
        img = filteringImage(img, typeBlur, tbs)
    except:
        print('problems in a type border select! quickly move the slider!!!')
    cv2.imshow('w2', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        #output = "th1=%d\nth2=%d\ndit=%d\neit=%d" % (th1, th2, dit, eit)
        #print(output)
        break
cv2.destroyAllWindows()