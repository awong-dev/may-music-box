#ifndef SONG_H_
#define SONG_H_

enum class SongColor : int {
  Invalid = -1,
  Red = 0,
  Orange = 1,
  Yellow = 2,
  Green = 3,
  Blue = 4,
  Purple = 5,
};

inline constexpr int kNumColors = 6;

#endif /* SONG_H_ */
