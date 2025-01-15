export const calcJoystickTop = (left, right) => {
  switch (true) {
    case left < 0 && right > 0:
    case left > 0 && right < 0:
      return 0;
    case left === right:
      return left;
    case Math.abs(left) > Math.abs(right):
      return right;
    case Math.abs(left) < Math.abs(right):
      return left;
    default:
      return 0;
  }
};

export const calcJoystickRight = (left, right) => {
  switch (true) {
    case left > 0 && right < 0:
      return 100;
    case left < 0 && right > 0:
      return -100;
    case left > right:
      return Math.sqrt(left * left - right * right);
    case left < right:
      return -Math.sqrt(right * right - left * left);
    default:
      return 0;
  }
};
