const FACES = ['A', 'B', 'C', 'D', 'E', 'F'] as const;
const LEVELS = ['L2', 'L3', 'L4'] as const;
const SIDES = ['LEFT', 'RIGHT'] as const;
const BRANCH_STATES = ['FREE', 'OCCUPIED', 'DISABLED'] as const;
export { FACES, LEVELS, SIDES, BRANCH_STATES };
type Face = typeof FACES[number];
type Level = typeof LEVELS[number];
type Side = typeof SIDES[number];
type BranchState = typeof BRANCH_STATES[number];
type Branch = { face: Face, level: Level, side: Side };
export type {Face, Level, Side, BranchState, Branch};
// export type ReefState = Record<Face, Record<Level, Record<Side, BranchState>>>;

export class ReefState {
  map: Record<Face, Record<Level, Record<Side, BranchState>>>;

  constructor() {
    this.map = FACES.reduce((reefAcc, face) => {
      reefAcc[face] = LEVELS.reduce((levelAcc, level) => {
        levelAcc[level] = SIDES.reduce((sideAcc, side) => {
          sideAcc[side] = 'FREE';
          return sideAcc;
        }, {} as Record<Side, BranchState>);
        return levelAcc;
      }, {} as Record<Level, Record<Side, BranchState>>);
      return reefAcc;
    }, {} as Record<Face, Record<Level, Record<Side, BranchState>>>);
  }
}
