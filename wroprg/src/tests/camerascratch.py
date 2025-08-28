import cv2
import glob
import os



def detect_border_distance(image)-> (float,float,float):
    # Detect the distance of the borders in the image
    left_distance = 0.0
    right_distance = 0.0
    front_distance = 0.0

    #divide the image into 1/4, 2/4 , 3/4 sections based on width
    height, width, _ = image.shape
    section_width = width // 4

    # Calculate distances for each section
    sections = []
    # left section
    left = image[:, :section_width]
    sections.append(left)
    cv2.imshow('Left Section', left)

    # center section
    center = image[:, section_width:3 * section_width]
    sections.append(center)
    cv2.imshow('Center Section', center)

    # right section
    right = image[:, 3 * section_width:]
    sections.append(right)
    cv2.imshow('Right Section', right)

    # for i, section in enumerate(sections):
        # cv2.imshow(f'Section {i + 1}', section)
        # left_distance, right_distance, top_distance = calculate_distances(section)
        # left_distance += left_distance
        # right_distance += right_distance
        # top_distance += top_distance

    #in the bottom 75% of center image find black percentage.
    bottom_image = center[height // 2:, :]
    black_pixels = cv2.inRange(bottom_image, (0, 0, 0), (50, 50, 50))
    black_percentage = (cv2.countNonZero(black_pixels) / (bottom_image.shape[0] * bottom_image.shape[1])) * 100
    front_distance = black_percentage
    print(f"Black percentage in bottom center section: {black_percentage:.2f}%")

    bottom_half = left[height // 2:, :]
    black_pixels = cv2.inRange(bottom_half, (0, 0, 0), (50, 50, 50))
    black_percentage = (cv2.countNonZero(black_pixels) / (bottom_half.shape[0] * bottom_half.shape[1])) * 100
    left_distance = black_percentage
    print(f"Black percentage in left section: {black_percentage:.2f}%")

    bottom_half = right[height // 2:, :]
    black_pixels = cv2.inRange(bottom_half, (0, 0, 0), (50, 50, 50))
    black_percentage = (cv2.countNonZero(black_pixels) / (bottom_half.shape[0] * bottom_half.shape[1])) * 100
    right_distance = black_percentage
    print(f"Black percentage in right section: {black_percentage:.2f}%")

    return left_distance, right_distance, front_distance

# Path to your image file
# Get all image file paths in the output folder
image_folder = 'C:\\Users\\saurabh\\Downloads\\data\outputfullrun'
#image_folder = 'C:\\Users\\saurabh\\Downloads\\data\output_round2'

image_paths = glob.glob(os.path.join(image_folder, '*.jpg'))

for image_path in image_paths:
    # Read the image in BGR format
    image = cv2.imread(image_path)
    img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Take the top half of the image
    height = img_rgb.shape[0]
    top_half = img_rgb[:height // 2, :]

    # Rotate the top half 180 degrees clockwise
    img_rgb = cv2.rotate(top_half, cv2.ROTATE_180)
    bottom_half = img_rgb
    # cv2.imshow('Half Image', bottom_half)
    img_rgb = bottom_half
    if image is None:
        print("Error: Image not found or unable to load.")
    else:
        # Display the image
        (f,l,r) = detect_border_distance(img_rgb)
        if f > 50 or l > 50 or r > 50:
            print(f"Front: {f}, Left: {l}, Right: {r}")
            cv2.imshow('RGB Image', img_rgb)
            print(f"Displayed image: {image_path}")
            if cv2.waitKey(0) == 27:
                break
cv2.destroyAllWindows()