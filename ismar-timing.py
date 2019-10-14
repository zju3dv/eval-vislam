import sys
import argparse
import subprocess
import re
import numpy as np

sigma_dict = {'VSLAM': {'APE': 72.46, 'RPE': 6.72, 'ARE': 7.41, 'RRE': 0.26, 'Badness': 20.68, 'InitQuality': 2.79, 'robustness': 2.27, 'Relocalization Time': 0.65},
              'VISLAM': {'APE': 55.83, 'RPE': 2.92, 'ARE': 2.48, 'RRE': 0.17, 'Badness': 2.38, 'InitQuality': 1.85, 'robustness': 0.95, 'Relocalization Time': 1.42}}


def get_value(res_str, key):
    reg_float = '\D*([-+]?[0-9]*\.?[0-9]+)'
    try:
        return float(re.search(r'' + key + reg_float, res_str).group(1))
    except:
        print('Error in get_value', key, '\nOutput : ', res_str)
        return -1


def get_score(res_str, method, key, verbose=True):
    raw_value = 0
    if key == 'Badness':
        raw_value = 100 - get_value(res_str, 'CMPL')
    elif key == 'InitQuality':
        raw_value = get_value(res_str, 'E_init')
    else:
        raw_value = get_value(res_str, key)
    sigma = sigma_dict[method][key]
    score = (sigma * sigma) / (sigma * sigma + raw_value * raw_value)
    if raw_value < 0:
        score = 0
    if verbose:
        print('---->', key, ': %.3f' % (raw_value) if raw_value >= 0 else ': invalid', ', Score :', "%.4f" % score)
    return score


def select_round(total_round, gt_folder, traj_folder):
    ape_list = []
    for i in range(total_round):
        traj_file = traj_folder + '/' + str(i) + '-pose.txt'
        try:
            text_result = subprocess.check_output(
                ['bin/accuracy', gt_folder, traj_file, fix_scale]).decode(sys.stdout.encoding)
            ape_list.append(get_score(text_result, method, 'APE', False))
        except Exception:
            print('\nFail to run: ', 'bin/accuracy', gt_folder, traj_file, fix_scale)
            return -1
    return ape_list.index(np.percentile(ape_list, 50, interpolation='nearest'))

# e.g. python3 ismar-score.py --round 5 --is_vislam 1 --trajectory_base_dir /path/to/your/trajectory --gt_base_dir /path/to/your/dataset/folder
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='ISMAR2019 SLAM Challenge Timing')
    parser.add_argument('--round', type=str)
    parser.add_argument('--is_vislam', type=str)
    parser.add_argument('--trajectory_base_dir', type=str)
    parser.add_argument('--gt_base_dir', type=str)
    parser.add_argument('--time_ratio', type=str, default='1.0')
    args = parser.parse_args()
    trajectory_base_dir = args.trajectory_base_dir
    gt_base_dir = args.gt_base_dir
    total_round = int(args.round)
    time_ratio = args.time_ratio
    is_vislam = True if args.is_vislam == '1' else '0'
    fix_scale = '1' if is_vislam == True else '0'
    method = 'VISLAM' if is_vislam == True else 'VSLAM'

    # test-round1
    accuracy_cmpl_init_eval_list = ['C1_test', 'C3_test', 'C5_test', 'C7_test', 'C9_test', 'C11_test', 'D8_test', 'D10_test']
    robustness_eval_list = ['D0_test', 'D3_test', 'D4_test']
    reloc_time_eval_list = ['D6_test']

    # test-full
    accuracy_cmpl_init_eval_list = ['C0_test', 'C1_test', 'C2_test', 'C3_test', 'C4_test', 'C5_test', 'C6_test', 'C7_test', 'C8_test', 'C9_test', 'C10_test', 'C11_test', 'D8_test', 'D9_test', 'D10_test']
    robustness_eval_list = ['D0_test', 'D1_test', 'D2_test', 'D3_test', 'D4_test']
    reloc_time_eval_list = ['D5_test', 'D6_test', 'D7_test']

    full_list = list(set(accuracy_cmpl_init_eval_list +
                         robustness_eval_list + reloc_time_eval_list))
    full_list.sort()

    slam_run_full_count = 0
    slam_run_full_time = 0.0

    for seq_name in full_list:
        print('------------------------------')
        has_static_segment = '1' if (seq_name.find('A') != -1 or seq_name.find('C') != -1) else '0'
        gt_folder = gt_base_dir + '/' + seq_name
        round = select_round(total_round, gt_folder,
                             trajectory_base_dir + '/' + seq_name)
        print('Sequence Name', seq_name, ', Select Round', round if round >= 0 else 'invalid')
        # invalid round detected
        if round < 0:
            print('invalid round detected!')
            continue
        time_file = trajectory_base_dir + '/' + \
            seq_name + '/' + str(round) + '-time.txt'
        
        text_result = subprocess.check_output(
            ['bin/extract_time', gt_folder, time_file]).decode(sys.stdout.encoding)
        slam_run_count = int(get_value(text_result, 'slam_run_count'))
        slam_run_time = get_value(text_result, 'slam_run_time')
        local_frame_rate = slam_run_count / slam_run_time
        print('Frame Rate : %.4f' %(local_frame_rate))
        slam_run_full_count += slam_run_count
        slam_run_full_time += slam_run_time
    if slam_run_full_count > 0:
        frame_rate = slam_run_full_count / slam_run_full_time
        print('Average Frame Rate : %.4f\nNon-Realtime Penalty Factor %.4f' % (frame_rate, min(30.0, frame_rate)/30.0))
    
