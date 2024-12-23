import cv2

def get_mouse_pos(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f'x: {x}, y: {y}')

def main():
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', get_mouse_pos)

    while True:
        image = cv2.imread('/home/jaenote/rokey_week7_ws/train/puple_34.jpg')
        cv2.imshow('image', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()