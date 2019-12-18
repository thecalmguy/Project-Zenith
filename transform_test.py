import numpy as np

def main():
    x = 320
    y = 240
    gps_center = (21.1, 91.6)
    heading = -30
    height = 5

    x0 = 640/2
    y0 = 480/2
    pix_to_m = 1*height/640
    theta_pixel = np.arctan2(x0-x, 480-y-y0)
    # print("Theta_Pixel: {0}".format(theta_pixel*180/np.pi))
    dist_pixel  = np.sqrt((x-x0)**2 + (480-y-y0)**2)
    theta       = heading*np.pi/180 + theta_pixel
    dist_met    = dist_pixel * pix_to_m
    x_met       = dist_met * np.cos(theta + np.pi/2)
    y_met       = dist_met * np.sin(theta + np.pi/2)
    print("{0}, {1}".format(x_met, y_met))
    lat         = gps_center[0] + y_met/102470#times something
    lon         = gps_center[1] + x_met/102470#times something

if __name__ == "__main__":
    main()
