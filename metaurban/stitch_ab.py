"""stitch_ab — side-by-side A/B mp4 + poster from two rendered laps (mirrors ab_runner.build_ab, generalized).
  python stitch_ab.py LEFT.mp4 "A label" RIGHT.mp4 "B label" OUT.mp4
The shorter clip freezes on its last frame so both run to the longer one's length."""
import sys
import numpy as np, cv2, imageio.v2 as imageio

left_mp4, left_lab, right_mp4, right_lab, out_mp4 = sys.argv[1:6]
ro = imageio.get_reader(left_mp4); rn = imageio.get_reader(right_mp4)
Na, Nb = ro.count_frames(), rn.count_frames()
N = max(Na, Nb)
fa = [ro.get_data(i) for i in range(Na)]; fb = [rn.get_data(i) for i in range(Nb)]
H = max(fa[0].shape[0], fb[0].shape[0]); gap = 8
pad = lambda fr: fr if fr.shape[0] == H else np.vstack([fr, np.zeros((H - fr.shape[0], fr.shape[1], 3), fr.dtype)])


def lab(fr, txt, col, done=False):
    fr = np.ascontiguousarray(pad(fr)); cv2.rectangle(fr, (0, 0), (430, 24), (0, 0, 0), -1)
    cv2.putText(fr, txt + ("  [reached]" if done else ""), (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA); return fr


wr = imageio.get_writer(out_mp4, fps=20, macro_block_size=8)
for i in range(N):
    a = lab(fa[min(i, Na - 1)], left_lab, (190, 190, 190), done=i >= Na)
    b = lab(fb[min(i, Nb - 1)], right_lab, (40, 230, 40), done=i >= Nb)
    wr.append_data(np.hstack([a, np.full((H, gap, 3), 30, np.uint8), b]))
wr.close()
i = min(N // 2, N - 1)
a = lab(fa[min(i, Na - 1)], left_lab, (190, 190, 190), done=i >= Na)
b = lab(fb[min(i, Nb - 1)], right_lab, (40, 230, 40), done=i >= Nb)
cv2.imwrite(out_mp4.replace(".mp4", "_poster.png"),
            np.hstack([a, np.full((H, gap, 3), 30, np.uint8), b])[..., ::-1])
print(f"[stitch] {out_mp4}  ({N} frames; left {Na} / right {Nb})", flush=True)
