import bpy
import copy
import hashlib
import json
import math
import struct
import sys
from pathlib import Path
from mathutils import Matrix, Quaternion, Vector

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'assets/UE5Manny_and_UEFNManny_Threepeat_Week4Anims'
WORK = ROOT / 'assets/threepeat-working'
OUT = ROOT / 'assets/threepeat-runtime'
DIAG = ROOT / 'diagnostics/threepeat-conversion'
for folder in (WORK, OUT, DIAG):
    folder.mkdir(parents=True, exist_ok=True)
SPECS = [(39, 'contextHang', 'BracedIdle', True),
         (40, 'contextHopLeft', 'BracedJumpLeftV5', False),
         (41, 'contextHopRight', 'BracedJumpRight', False),
         (42, 'contextMantle', 'BracedIdleMantleV7final', False)]
SK = json.loads((ROOT / 'assets/skyrim-skeleton.json').read_text())
B = SK['bones'][:99]
P = [b['parent'] for b in B]
REST = [copy.deepcopy(b['transform']) for b in B]
C = Quaternion((0, 0, 1), math.pi)


def q(a):
    return Quaternion((a[3], a[0], a[1], a[2])).normalized()


def qa(a):
    return [a.x, a.y, a.z, a.w]


def mat(t):
    return Matrix.LocRotScale(Vector(t['t']), q(t['q']), Vector(t.get('s', [1, 1, 1])))


def worlds(p):
    result = []
    for i, t in enumerate(p):
        result.append((result[P[i]] if P[i] >= 0 else Matrix.Identity(4)) @ mat(t))
    return result


TW = worlds(REST)
MAP = {4: 'pelvis', 5: 'pelvis', 24: 'spine_01', 25: 'spine_03',
       26: 'spine_05', 35: 'neck_02', 36: 'head'}
for side, indices in [('l', [6, 7, 8, 50, 27, 28, 29, 38]),
                      ('r', [9, 10, 11, 51, 30, 31, 32, 39])]:
    for i, name in zip(indices, ['thigh', 'calf', 'foot', 'ball', 'clavicle', 'upperarm', 'lowerarm', 'hand']):
        MAP[i] = name + '_' + side
    base = 67 if side == 'l' else 82
    for digit, name in enumerate(['thumb', 'index', 'middle', 'ring', 'pinky']):
        for segment in range(3):
            MAP[base + digit * 3 + segment] = f'{name}_{segment + 1:02}_{side}'
CHILD = {6: 7, 7: 8, 8: 50, 9: 10, 10: 11, 11: 51,
         27: 28, 28: 29, 29: 38, 30: 31, 31: 32, 32: 39}


def extract(name, source):
    path = SOURCE / f'UE5Manny_Threepeat_{source}.FBX'
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    cache = WORK / (name + '.json')
    if cache.exists() and '--resample' not in sys.argv:
        cached = json.loads(cache.read_text())
        if cached.get('sourceSHA256') == sha:
            return cached
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=str(path), use_anim=True)
    rig = next(ob for ob in bpy.context.scene.objects if ob.type == 'ARMATURE')
    scene = bpy.context.scene
    action = rig.animation_data.action
    first, last = map(float, action.frame_range)
    native_fps = scene.render.fps / scene.render.fps_base
    seconds = (last - first) / native_fps
    names = [bone.name for bone in rig.data.bones]
    rest, tails = [], []
    for bone in rig.data.bones:
        world = rig.matrix_world @ bone.matrix_local
        rest.append({'t': list(world.translation), 'q': qa(world.to_quaternion())})
        tails.append(list(rig.matrix_world @ bone.tail_local))
    frames = []
    count = round(seconds * 60)
    for index in range(count + 1):
        at = first + (last - first) * index / count
        scene.frame_set(int(at), subframe=at - int(at))
        frames.append([{'t': list((rig.matrix_world @ bone.matrix).translation),
                        'q': qa((rig.matrix_world @ bone.matrix).to_quaternion())}
                       for bone in rig.pose.bones])
    result = {'source': path.name, 'sourceSHA256': sha, 'bones': names,
              'rest': rest, 'restTails': tails, 'frames': frames, 'fps': 60,
              'nativeFPS': native_fps, 'sourceRange': [first, last], 'seconds': seconds}
    cache.write_text(json.dumps(result, separators=(',', ':')), encoding='utf-8')
    print('EXTRACTED', name, len(frames), 'at', native_fps, 'native fps', flush=True)
    return result


def anatomical_frame(direction, normal):
    z = direction.normalized()
    y = normal - z * normal.dot(z)
    if y.length < 1e-5:
        raise ValueError('Degenerate anatomical frame')
    y.normalize()
    return Matrix((y.cross(z).normalized(), y, z)).transposed().to_quaternion()


def calibrate(data):
    sr = data['rest']
    index = {name: i for i, name in enumerate(data['bones'])}
    calibrated = {}
    for i in range(99):
        basis = TW[i].to_quaternion()
        if P[i] >= 0:
            basis = calibrated[P[i]] @ q(REST[i]['q'])
        if i in CHILD:
            child = CHILD[i]
            direction = (basis @ Vector(REST[child]['t'])).normalized()
            source = C @ (Vector(sr[index[MAP[child]]]['t']) - Vector(sr[index[MAP[i]]]['t'])).normalized()
            basis = direction.rotation_difference(source) @ basis
        calibrated[i] = basis



    for side, hand, base in [('l', 38, 67), ('r', 39, 82)]:
        def src(name):
            return C @ Vector(sr[index[name + '_' + side]]['t'])
        wrist = src('hand')
        source_normal = (src('index_01') - wrist).cross(src('pinky_01') - wrist).normalized()
        target_wrist = TW[hand].translation
        target_normal = (TW[base + 3].translation - target_wrist).cross(TW[base + 12].translation - target_wrist).normalized()
        calibrated[hand] = anatomical_frame(src('middle_01') - wrist, source_normal) @ anatomical_frame(TW[base + 6].translation - target_wrist, target_normal).inverted() @ TW[hand].to_quaternion()
        for digit, finger in enumerate(['thumb', 'index', 'middle', 'ring', 'pinky']):
            for segment in range(3):
                bone = base + digit * 3 + segment
                name = f'{finger}_{segment + 1:02}'
                if segment < 2:
                    direction = src(f'{finger}_{segment + 2:02}') - src(name)
                    target = TW[bone + 1].translation - TW[bone].translation
                else:





                    source_bone = index[name + '_' + side]
                    direction = C @ q(sr[source_bone]['q']) @ Vector((1 if side == 'l' else -1, 0, 0))
                    target = TW[bone].to_quaternion() @ Vector((0, 0, 1))
                calibrated[bone] = anatomical_frame(direction, source_normal) @ anatomical_frame(target, target_normal).inverted() @ TW[bone].to_quaternion()
    scale = ((TW[7].translation - TW[6].translation).length + (TW[8].translation - TW[7].translation).length) / ((Vector(sr[index['calf_l']]['t']) - Vector(sr[index['thigh_l']]['t'])).length + (Vector(sr[index['foot_l']]['t']) - Vector(sr[index['calf_l']]['t'])).length)
    return index, calibrated, scale


def retarget(data):
    index, calibrated, scale = calibrate(data)
    sr = data['rest']
    frames = data['frames']
    origin = C @ Vector(frames[0][index['pelvis']]['t']) * scale
    roots = [C @ Vector(frame[index['pelvis']]['t']) * scale - origin for frame in frames]
    poses = []
    for frame, root in zip(frames, roots):
        pose = copy.deepcopy(REST)
        pose[4]['t'] = list(Vector(REST[4]['t']) + root)
        computed = []
        for i, tr in enumerate(pose):
            parent = computed[P[i]] if P[i] >= 0 else Matrix.Identity(4)
            if i in MAP:
                source = index[MAP[i]]
                delta = C @ q(frame[source]['q']) @ q(sr[source]['q']).inverted() @ C.inverted()
                desired = delta @ calibrated[i]
                if i == 5:
                    desired = computed[4].to_quaternion() @ q(REST[5]['q'])
                tr['q'] = qa((parent.to_quaternion().inverted() @ desired).normalized())
            computed.append(parent @ mat(tr))
        poses.append(pose)
    return poses, roots, scale


def smooth(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def palm(w, side):
    hand, base = (38, 67) if side == 0 else (39, 82)
    return w[hand].translation * .5 + sum((w[base + j].translation for j in (3, 6, 9, 12)), Vector()) * .125


def palm_info(pose):
    w = worlds(pose)
    hands = [palm(w, hand) for hand in (0, 1)]
    return {'left': list(hands[0]), 'right': list(hands[1]),
            'midpoint': list((hands[0] + hands[1]) * .5),
            'halfWidth': abs(hands[1].x - hands[0].x) * .5}


def path_knots(roots, adapt_out=False):
    travel = roots[-1]
    rows = []
    for i in range(0, len(roots), 3):
        progress = roots[i].x / travel.x
        out = -(roots[i].y - travel.y * progress)
        if adapt_out:



            out = 0 if out <= 0 else out*out if out < .5 else out-.25
        rows.append([i / (len(roots) - 1), progress,
                     roots[i].z - travel.z * progress, out])
    assert rows[-1][0] == 1
    return rows


def sample_path(rows, phase):
    phase = max(0.0, min(1.0, phase))
    segment = min(int(phase * (len(rows) - 1)), len(rows) - 2)
    a, b = rows[segment], rows[segment + 1]
    before, after = rows[max(0, segment - 1)], rows[min(len(rows) - 1, segment + 2)]
    width = b[0] - a[0]
    t = (phase - a[0]) / width
    result = []
    for slot in (1, 2, 3):
        da = (b[slot] - before[slot]) / (b[0] - before[0])
        db = (after[slot] - a[slot]) / (after[0] - a[0])
        if slot == 3:
            slope = (b[slot] - a[slot]) / width
            if abs(slope) < 1e-8:
                da = db = 0
            else:
                da = 0 if da*slope <= 0 else math.copysign(min(abs(da), 3*abs(slope)), slope)
                db = 0 if db*slope <= 0 else math.copysign(min(abs(db), 3*abs(slope)), slope)
        result.append((2 * t**3 - 3 * t*t + 1) * a[slot] + (t**3 - 2*t*t + t) * width * da + (-2*t**3 + 3*t*t) * b[slot] + (t**3 - t*t) * width * db)
    return result




HOP_EVENTS = {
    'contextHopLeft': {'sourceRelease': [[.03, .10], [.13, .23]],
                       'targetCatch': [[.50, .66], [.61, .71]]},
    'contextHopRight': {'sourceRelease': [[.12, .24], [.025, .11]],
                        'targetCatch': [[.64, .73], [.53, .69]]},
}



MANTLE_RELEASE = [[.84, .97], [.93, 1.05]]


def contact_weights(name, phase, seconds):
    at = phase * seconds
    if name == 'contextHang':
        return [1, 1, 0, 0], [1, 1], [0, 0]
    if name == 'contextMantle':
        plant = 1-smooth((at-.30)/.09)+smooth((at-.62)/.08)
        hands = [plant * (1 - smooth((at - begin) / (end - begin))) for begin, end in MANTLE_RELEASE]
        return hands + [0, 0], hands, [0, 0]
    events = HOP_EVENTS[name]
    source = [1 - smooth((at - begin) / (end - begin)) for begin, end in events['sourceRelease']]
    target = [smooth((at - begin) / (end - begin)) for begin, end in events['targetCatch']]
    return [min(1, a + b) for a, b in zip(source, target)] + [0, 0], source, target


def smooth_rotations(poses, loop):
    original = copy.deepcopy(poses)
    for frame, pose in enumerate(poses):
        for i, tr in enumerate(pose):
            rotation = q(original[frame][i]['q'])
            total = 1.0
            for delta, weight in [(-2, .12), (-1, .35), (1, .35), (2, .12)]:
                index = (frame + delta) % (len(poses) - 1) if loop else max(0, min(len(poses) - 1, frame + delta))
                rotation = rotation.slerp(q(original[index][i]['q']), weight / (total + weight))
                total += weight
            strength = 1.0 if loop else smooth(min(frame, len(poses) - 1 - frame) / 3)
            tr['q'] = qa(q(original[frame][i]['q']).slerp(rotation, strength).normalized())
    if loop:
        poses[-1] = copy.deepcopy(poses[0])


def forearm_helpers(poses):


    for pose in poses:
        for hand, helpers in [(38, (52, 53)), (39, (56, 57))]:
            axis = Vector(REST[hand]['t']).normalized()
            delta = q(pose[hand]['q']) @ q(REST[hand]['q']).inverted()
            if delta.w < 0:
                delta.negate()
            along = axis * Vector((delta.x, delta.y, delta.z)).dot(axis)
            twist = Quaternion((delta.w, *along))
            twist = twist.normalized() if twist.magnitude > 1e-6 else Quaternion()
            for bone in helpers:
                fraction = max(0, min(1, Vector(REST[bone]['t']).length / Vector(REST[hand]['t']).length))
                pose[bone]['q'] = qa(Quaternion().slerp(twist, fraction) @ q(REST[bone]['q']))


def protect_wrists(poses):
    original = copy.deepcopy(poses)
    def project(pose, soft):
        w = worlds(pose)
        for elbow, hand, middle in [(29, 38, 73), (32, 39, 88)]:
            arm = (w[hand].translation - w[elbow].translation).normalized()
            palm_axis = (w[middle].translation - w[hand].translation).normalized()
            angle = arm.angle(palm_axis)
            degrees = math.degrees(angle)
            target = 80 + 15 * math.tanh((degrees - 80) / 15) if soft and degrees > 80 else min(degrees, 95)
            if target < degrees - 1e-5:
                delta = palm_axis.rotation_difference(arm)
                correction = Quaternion().slerp(delta, (degrees - target) / degrees)
                pose[hand]['q'] = qa((w[elbow].to_quaternion().inverted() @ correction @ w[hand].to_quaternion()).normalized())
    for pose in poses:
        project(pose, True)
    angular_limit = math.radians(12)
    for _ in range(10):
        for sequence in (range(1, len(poses)-1), range(len(poses)-2, 0, -1)):
            forward = sequence.step > 0
            for index in sequence:
                neighbor = index - 1 if forward else index + 1
                for hand in (38, 39):
                    previous = q(poses[neighbor][hand]['q'])
                    desired = q(poses[index][hand]['q'])
                    angle = 2 * math.acos(min(1.0, abs(previous.dot(desired))))
                    if angle > angular_limit:
                        poses[index][hand]['q'] = qa(previous.slerp(desired, angular_limit / angle))
        for pose in poses:
            project(pose, False)
    correction = max(math.degrees(2 * math.acos(min(1.0, abs(q(a[hand]['q']).dot(q(b[hand]['q'])))))) for a, b in zip(original, poses) for hand in (38, 39))
    maximum = max(math.degrees(2 * math.acos(min(1.0, abs(q(a[hand]['q']).dot(q(b[hand]['q'])))))) for a, b in zip(poses, poses[1:]) for hand in (38, 39))
    bend = 0.0
    for pose in poses:
        w = worlds(pose)
        for elbow, hand, middle in [(29, 38, 73), (32, 39, 88)]:
            bend = max(bend, math.degrees((w[hand].translation-w[elbow].translation).angle(w[middle].translation-w[hand].translation)))
    assert bend <= 95.001, ('Wrist bend limit failed', bend)
    assert maximum <= 12.01, ('Wrist angular step limit failed', maximum)
    return {'method': 'anatomical palm-axis minimal swing, 80 degree soft knee / 95 degree maximum, bidirectional local wrist velocity limit',
            'maxCorrectionDegrees': correction, 'maxWristStepDegrees': maximum, 'maxWristBendDegrees': bend}


def pack_transform(tr):
    return struct.pack('<10f', *(tr['t'] + tr['q'] + tr['s']))


def write_header(clips):
    lines = ['#pragma once', '#include <algorithm>', '#include <array>',
             '',
             'namespace fc {', 'inline constexpr float threepeatHopSeconds=1.3f;',
             'inline constexpr float threepeatMantleSeconds=1.8666666667f;',
             'inline float threepeatEase(float t){t=std::clamp(t,0.f,1.f);return t*t*(3-2*t);}',
             'struct ThreepeatPathKnot { float phase,travel,lift,out; };']
    def ff(value):
        return f'{value:.10f}f'
    for clip in clips[1:3]:
        name = 'threepeatLeftPath' if clip['name'] == 'contextHopLeft' else 'threepeatRightPath'
        rows = clip['bodyPathKnots']
        lines.append(f'inline constexpr std::array<ThreepeatPathKnot,{len(rows)}> {name}{{{{')
        lines.extend('    {' + ','.join(ff(x) for x in row) + '},' for row in rows)
        lines.append('}};')
    lines += [
        'inline float threepeatPathValue(bool left,float phase,int slot) {',
        '    const auto& rows=left?threepeatLeftPath:threepeatRightPath;',
        '    phase=std::clamp(phase,0.f,1.f);',
        '    const int i=std::min(int(phase*float(rows.size()-1)),int(rows.size())-2);',
        '    const auto a=rows[i],b=rows[i+1],before=rows[std::max(0,i-1)],after=rows[std::min(int(rows.size())-1,i+2)];',
        '    const auto value=[&](ThreepeatPathKnot k){return slot==0?k.travel:slot==1?k.lift:k.out;};',
        '    const float width=b.phase-a.phase,t=(phase-a.phase)/width;',
        '    float da=(value(b)-value(before))/(b.phase-before.phase),db=(value(after)-value(a))/(after.phase-a.phase);',
        '    if(slot==2){const float slope=(value(b)-value(a))/width;',
        '        if(slope>-1e-8f&&slope<1e-8f)da=db=0;',
        '        else {const float bound=slope>0?3*slope:-3*slope;',
        '            da=da*slope<=0?0:std::clamp(da,-bound,bound);db=db*slope<=0?0:std::clamp(db,-bound,bound);}}',
        '    return (2*t*t*t-3*t*t+1)*value(a)+(t*t*t-2*t*t+t)*width*da+(-2*t*t*t+3*t*t)*value(b)+(t*t*t-t*t)*width*db;',
        '}',
        'inline float threepeatHopTravel(bool left,float phase){return threepeatPathValue(left,phase,0);}',
        'inline float threepeatHopLift(bool left,float phase){return threepeatPathValue(left,phase,1);}',
        'inline float threepeatHopOut(bool left,float phase){return threepeatPathValue(left,phase,2);}',
        'inline float threepeatSourceWeight(bool left,int hand,float phase){',
        '    const float at=phase*threepeatHopSeconds;',
        '    const float begin=left?(hand==0?.03f:.13f):(hand==0?.12f:.025f);',
        '    const float end=left?(hand==0?.10f:.23f):(hand==0?.24f:.11f);',
        '    return 1-threepeatEase((at-begin)/(end-begin));',
        '}',
        'inline float threepeatTargetWeight(bool left,int hand,float phase){',
        '    const float at=phase*threepeatHopSeconds;',
        '    const float begin=left?(hand==0?.50f:.61f):(hand==0?.64f:.53f);',
        '    const float end=left?(hand==0?.66f:.71f):(hand==0?.73f:.69f);',
        '    return threepeatEase((at-begin)/(end-begin));',
        '}',
        'inline float threepeatMantleWeight(int hand,float phase){',
        '    const float begin=hand==0?.84f:.93f,end=hand==0?.97f:1.05f;',
        '    const float at=phase*threepeatMantleSeconds;',
        '    const float plant=1-threepeatEase((at-.30f)/.09f)+threepeatEase((at-.62f)/.08f);',
        '    return plant*(1-threepeatEase((at-begin)/(end-begin)));',
        '}', '}', '']
    (ROOT / 'src/ThreepeatMotion.h').write_text('\n'.join(lines), encoding='utf-8')


def finish(sources):
    preliminary = [retarget(data) for data in sources]
    for poses, roots, _ in preliminary:
        assert all(len(pose) == 99 for pose in poses)
    hang_info = palm_info(preliminary[0][0][0])
    shared_offset = Vector((-hang_info['midpoint'][0], 33 - hang_info['midpoint'][1], 0))
    clips = []
    for (motion, name, _, loop), data, (poses, roots, scale) in zip(SPECS, sources, preliminary):
        travel = roots[-1].copy()
        offset = shared_offset.copy()
        if name == 'contextMantle':
            original = palm_info(poses[0])
            offset = Vector((0, 33, hang_info['midpoint'][2])) - Vector(original['midpoint'])


        for pose in poses:
            pose[4]['t'] = list(Vector(pose[4]['t']) + offset)
        raw_rows = path_knots(roots) if name in HOP_EVENTS else None
        rows = path_knots(roots, True) if name in HOP_EVENTS else None
        residual = []
        if rows:
            for i, pose in enumerate(poses):
                progress, lift, out = sample_path(raw_rows, i / (len(poses) - 1))
                actor = Vector((travel.x * progress, travel.y * progress - out, travel.z * progress + lift))
                pose[4]['t'] = list(Vector(pose[4]['t']) - actor)
                residual.append(list(roots[i] - actor))
        elif loop:
            for i, pose in enumerate(poses):
                pose[4]['t'] = list(Vector(pose[4]['t']) - travel * (i / (len(poses) - 1)))
        smooth_rotations(poses, loop)
        wrist_safety = protect_wrists(poses)
        forearm_helpers(poses)
        weights, source_weights, target_weights = [], [], []
        for i in range(len(poses)):
            weight, source, target = contact_weights(name, i / (len(poses) - 1), data['seconds'])
            weights.append(weight)
            source_weights.append(source)
            target_weights.append(target)
        height = 0.0
        root_travel = travel.copy()
        if name == 'contextMantle':
            end = worlds(poses[-1])
            height = min(end[8].translation.z, end[11].translation.z) - 6


            travel = Vector((root_travel.x, (end[8].translation.y + end[11].translation.y) * .5, height))
        clip = {'id': motion, 'name': name, 'source': data['source'], 'sourceSHA256': data['sourceSHA256'],
                'sourceRange': data['sourceRange'], 'seconds': data['seconds'], 'stride': 0.0,
                'height': height, 'travel': list(travel), 'rawRootTravel': list(root_travel),
                'frames': poses, 'contacts': weights, 'loop': loop, 'sampleFPS': 60,
                'sourceNativeFPS': data['nativeFPS'], 'unitScale': scale, 'bodyAlignment': list(offset),
                'firstPalms': palm_info(poses[0]), 'lastPalms': palm_info(poses[-1]),
                'bodyPathKnots': rows, 'bodyPathResidual': residual,
                'bodyPathOriginalKnots': raw_rows,
                'sourceContacts': source_weights, 'targetContacts': target_weights,
                'contactEventsSeconds': HOP_EVENTS.get(name, {'sourceRelease': MANTLE_RELEASE} if name == 'contextMantle' else None),
                'contactMethod': 'separately reviewed source/destination hand windows; feet not artificially planted',
                'rootConvention': 'captured COM minus original actor path; capsule AND complete body share outward adaptation; bounded runtime arms must revalidate real contacts' if rows else 'full captured COM root' if name == 'contextMantle' else 'closed stationary source cycle',
                'wristSafety': wrist_safety,
                'validationScope': 'offline skeletal retarget, not Skyrim skin or in-game validation'}
        clip['maxJointStepDegrees'] = max(math.degrees(2*math.acos(min(1.0, abs(q(a['q']).dot(q(b['q'])))))) for p, r in zip(poses, poses[1:]) for a, b in zip(p, r))
        clip['maxRootResidual'] = max((Vector(v).length for v in residual), default=0)
        if rows:
            path_check = [sample_path(rows, k/600) for k in range(601)]
            assert min(row[2] for row in path_check) >= -1e-6, 'Checked body path enters the wall'
            clip['bodyPathRanges'] = {label: [min(row[j] for row in path_check), max(row[j] for row in path_check)]
                                      for j, label in enumerate(('progress', 'lift', 'out'))}
            clip['maxWholeBodyOutwardAdaptation'] = max(sample_path(rows, k/600)[2]-sample_path(raw_rows, k/600)[2] for k in range(601))
            clip['bodyPathReconstructionTolerance'] = 1e-4
            for k, root in enumerate(roots):
                progress, lift, out = sample_path(raw_rows, k/(len(roots)-1))
                reconstructed = Vector((root_travel.x*progress, root_travel.y*progress-out, root_travel.z*progress+lift)) + Vector(residual[k])
                assert (root-reconstructed).length < 1e-4, 'Original path / COM residual mismatch'
        clips.append(clip)
        print('FINAL', name, 'palms', clip['firstPalms'], 'height', height,
              'travel', list(travel), 'rootResidual', clip['maxRootResidual'], flush=True)
    write_header(clips)
    skeleton = b''
    for bone, transform in zip(B, REST):
        encoded = bone['name'].encode()
        skeleton += struct.pack('<iI', bone['parent'], len(encoded)) + encoded + pack_transform(transform)
    manifest = {'format': 'FCM4/version2/99bones/append-only',
                'source': 'Threepeat Games, Week 4; user-provided FBX, derived runtime clips only',
                'skeletonSHA256': hashlib.sha256((ROOT / 'assets/skyrim-skeleton.json').read_bytes()).hexdigest(),
                'skeletonBinarySHA256': hashlib.sha256(skeleton).hexdigest(),
                'converterSHA256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'header': 'src/ThreepeatMotion.h', 'clips': []}
    for clip in clips:
        name = clip['name']
        for pose in clip['frames']:
            for transform in pose:
                assert all(math.isfinite(x) for field in ('t', 'q', 's') for x in transform[field])
                assert abs(q(transform['q']).magnitude - 1) < 1e-4
        (OUT / (name + '.json')).write_text(json.dumps(clip, separators=(',', ':')), encoding='utf-8')
        chunk = struct.pack('<fI5f', clip['seconds'], len(clip['frames']), clip['stride'], clip['height'], *clip['travel'])
        for pose, contacts in zip(clip['frames'], clip['contacts']):
            chunk += b''.join(pack_transform(tr) for tr in pose) + struct.pack('<4f', *contacts)
        (OUT / (name + '.fcm4chunk')).write_bytes(chunk)
        info = {key: value for key, value in clip.items() if key not in ('frames', 'contacts', 'sourceContacts', 'targetContacts', 'bodyPathResidual')}
        info.update(file=name + '.json', chunk=name + '.fcm4chunk', sha256=hashlib.sha256(chunk).hexdigest(), frames=len(clip['frames']),
                    chunkSHA256=hashlib.sha256(chunk).hexdigest(),
                    jsonSHA256=hashlib.sha256((OUT / (name + '.json')).read_bytes()).hexdigest())
        manifest['clips'].append(info)
    (OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    (DIAG / 'retarget-metrics.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print('THREEPEAT_CONVERSION_COMPLETE', len(clips), flush=True)


if __name__ == '__main__':
    sources = [extract(name, source) for _, name, source, _ in SPECS]
    if '--extract-only' in sys.argv:
        print('EXTRACTION_COMPLETE', flush=True)
    else:
        finish(sources)
