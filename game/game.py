import cv2
import numpy as np
from ultralytics import YOLO

# Load your YOLO gesture detection model
model = YOLO("best (3).pt")

# Define the Tic-Tac-Toe board (3x3 grid)
board = np.zeros((3, 3), dtype=int)  # 0: empty, 1: player X, 2: player O
current_player = 1  # Start with player X

# Function to draw the Tic-Tac-Toe grid
def draw_board(img):
    height, width, _ = img.shape
    # Draw grid lines
    cv2.line(img, (width // 3, 0), (width // 3, height), (255, 255, 255), 3)
    cv2.line(img, (2 * width // 3, 0), (2 * width // 3, height), (255, 255, 255), 3)
    cv2.line(img, (0, height // 3), (width, height // 3), (255, 255, 255), 3)
    cv2.line(img, (0, 2 * height // 3), (width, 2 * height // 3), (255, 255, 255), 3)

# Function to place an X or O on the board
def draw_x_o(img, board):
    height, width, _ = img.shape
    for i in range(3):
        for j in range(3):
            center_x = int(j * width // 3 + width // 6)
            center_y = int(i * height // 3 + height // 6)
            if board[i, j] == 2:
                cv2.putText(img, 'X', (center_x - 50, center_y + 50), cv2.FONT_HERSHEY_SIMPLEX, 4, (255, 0, 0), 5)
            elif board[i, j] == 1:
                cv2.putText(img, 'O', (center_x - 50, center_y + 50), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 255, 0), 5)

# Function to map hand position (bounding box) to a Tic-Tac-Toe cell
def map_box_to_grid(box, frame_width, frame_height):
    # Get the center of the bounding box
    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2

    # Calculate which cell the center of the hand is in (3x3 grid)
    grid_col = center_x // (frame_width // 3)  # Column (0, 1, 2)
    grid_row = center_y // (frame_height // 3)  # Row (0, 1, 2)

    return grid_row, grid_col

# Function to detect gestures using YOLO model and map to Tic-Tac-Toe move
def detect_gesture_move(frame, model, current_player):
    frame_height, frame_width, _ = frame.shape
    # Run gesture detection using YOLO model
    results = model.predict(frame, imgsz=640, conf=0.544)

    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])  # Get the class ID of the detected gesture
            # Check if the detected gesture matches the current player's move
            if (current_player == 1 and class_id == 0):  # Class ID 0 for 'X' gesture
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                grid_row, grid_col = map_box_to_grid(box, frame_width, frame_height)
                return grid_row, grid_col
            elif (current_player == 2 and class_id == 1):  # Class ID 1 for 'O' gesture
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                grid_row, grid_col = map_box_to_grid(box, frame_width, frame_height)
                return grid_row, grid_col

    return None, None


def check_win(board):
    # Check rows, columns, and diagonals
    for i in range(3):
        if np.all(board[i, :] == current_player) or np.all(board[:, i] == current_player):
            return True
    if (board[0, 0] == board[1, 1] == board[2, 2] == current_player) or \
       (board[0, 2] == board[1, 1] == board[2, 0] == current_player):
        return True
    return False

def check_draw(board):
    return np.all(board != 0)



def display_winner_message(img, winner):
    height, width, _ = img.shape
    message = f"Player {winner} wins!" if winner != 'Draw' else "It's a draw!"
    cv2.putText(img, message, (width // 4, height // 3), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(img, "Press any key to exit", (width // 4, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.imshow('Tic-Tac-Toe', img)
    cv2.waitKey(3000)  # Wait for 3 seconds to allow users to read the message

# Start the camera feed
cap = cv2.VideoCapture(0)

# Create a named window for resizing
cv2.namedWindow('Tic-Tac-Toe', cv2.WINDOW_NORMAL)

# Set a custom window size, e.g., 1024x768
cv2.resizeWindow('Tic-Tac-Toe', 1024, 768)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Draw the Tic-Tac-Toe grid
    draw_board(frame)

    # Draw Xs and Os on the frame based on the current state of the board
    draw_x_o(frame, board)

    # Detect a move based on gesture recognition
    move_row, move_col = detect_gesture_move(frame, model, current_player)

    # If a valid move was detected and the cell is empty
    if move_row is not None and move_col is not None:
        if board[move_row, move_col] == 0:
            board[move_row, move_col] = current_player  # Update the board with the current player's move

            # Check for win or draw
            if check_win(board):
                display_winner_message(frame, current_player)
                break
            if check_draw(board):
                display_winner_message(frame, 'Draw')
                break

            # Switch players
            current_player = 3 - current_player  # Toggles between 1 and 2

    # Display the camera feed with the Tic-Tac-Toe grid and moves
    cv2.imshow('Tic-Tac-Toe', frame)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()
