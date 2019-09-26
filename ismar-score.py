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


def handle_invalid_sequence(accuracy_cmpl_init_eval_list, robustness_eval_list, reloc_time_eval_list, seq_name, score_list):
        if seq_name in accuracy_cmpl_init_eval_list:
            score_list['APE'].append(0)
            score_list['RPE'].append(0)
            score_list['ARE'].append(0)
            score_list['RRE'].append(0)
            score_list['Badness'].append(0)
            score_list['InitQuality'].append(0)
            print('---->', 'APE invalid, Score : 0')
            print('---->', 'RPE invalid, Score : 0')
            print('---->', 'ARE invalid, Score : 0')
            print('---->', 'RRE invalid, Score : 0')
            print('---->', 'Badness invalid, Score : 0')
            print('---->', 'InitQuality invalid, Score : 0')
        if seq_name in robustness_eval_list:
            score_list['Robustness'].append(0)
            print('---->', 'Robustness invalid, Score : 0')
        if seq_name in reloc_time_eval_list:
            score_list['Relocalization Time'].append(0)
            print('---->', 'Relocalization Time invalid, Score : 0')
        return score_list


# e.g. python3 ismar-score.py --round 5 --is_vislam 1 --trajectory_base_dir /path/to/your/trajectory --gt_base_dir /path/to/your/dataset/folder
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='ISMAR2019 SLAM Challenge Scoring')
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

    accuracy_cmpl_init_eval_list = ['C0_train', 'C1_train', 'C2_train', 'C3_train', 'C4_train', 'C5_train',
                                    'C6_train', 'C7_train', 'C8_train', 'C9_train', 'C10_train', 'C11_train', 'D8_train', 'D9_train', 'D10_train']
    robustness_eval_list = ['D0_train', 'D1_train',
                            'D2_train', 'D3_train', 'D4_train']
    reloc_time_eval_list = ['D5_train', 'D6_train', 'D7_train']

    # test-round1
    accuracy_cmpl_init_eval_list = ['C1_test', 'C3_test', 'C5_test', 'C7_test', 'C9_test', 'C11_test', 'D8_test', 'D10_test']
    robustness_eval_list = ['D0_test', 'D3_test', 'D4_test']
    reloc_time_eval_list = ['D6_test']

    full_list = list(set(accuracy_cmpl_init_eval_list +
                         robustness_eval_list + reloc_time_eval_list))
    full_list.sort()

    score_list = {'APE': [], 'RPE': [], 'ARE': [], 'RRE': [], 'Badness': [
    ], 'InitQuality': [], 'Robustness': [], 'Relocalization Time': []}

    for seq_name in full_list:
        print('------------------------------')
        has_static_segment = '1' if (seq_name.find('A') != -1 or seq_name.find('C') != -1) else '0'
        gt_folder = gt_base_dir + '/' + seq_name
        round = select_round(total_round, gt_folder,
                             trajectory_base_dir + '/' + seq_name)
        print('Sequence Name', seq_name, ', Select Round', round if round >= 0 else 'invalid')
        # invalid round detected
        if round < 0:
            score_list = handle_invalid_sequence(accuracy_cmpl_init_eval_list, robustness_eval_list, reloc_time_eval_list, seq_name, score_list)
            continue
        traj_file = trajectory_base_dir + '/' + \
            seq_name + '/' + str(round) + '-pose.txt'
        time_file = trajectory_base_dir + '/' + \
            seq_name + '/' + str(round) + '-time.txt'
        # accuracy, completeness, init quality
        if seq_name in accuracy_cmpl_init_eval_list:
            text_result = subprocess.check_output(
                ['bin/accuracy', gt_folder, traj_file, fix_scale]).decode(sys.stdout.encoding)
            text_result += subprocess.check_output(
                ['bin/initialization', gt_folder, traj_file, fix_scale, has_static_segment]).decode(sys.stdout.encoding)
            score_list['APE'].append(get_score(text_result, method, 'APE'))
            score_list['RPE'].append(get_score(text_result, method, 'RPE'))
            score_list['ARE'].append(get_score(text_result, method, 'ARE'))
            score_list['RRE'].append(get_score(text_result, method, 'RRE'))
            score_list['Badness'].append(
                get_score(text_result, method, 'Badness'))
            score_list['InitQuality'].append(
                get_score(text_result, method, 'InitQuality'))
        # robustness
        if seq_name in robustness_eval_list:
            text_result = subprocess.check_output(
                ['bin/robustness', gt_folder, traj_file, fix_scale]).decode(sys.stdout.encoding)
            score_list['Robustness'].append(
                get_score(text_result, method, 'robustness'))
        # relocalization time
        if seq_name in reloc_time_eval_list:
            text_result = subprocess.check_output(
                ['bin/relocalization_runtime', gt_folder, traj_file, time_file, fix_scale, '0.05', time_ratio]).decode(sys.stdout.encoding)
            score_list['Relocalization Time'].append(
                get_score(text_result, method, 'Relocalization Time'))
    single_scores = {}
    print('------------------------------')
    print('----------Summary-------------')
    for critera, score in score_list.items():
        avg = sum(score) / len(score)
        single_scores[critera] = avg
        print(critera, ' : %.4f' % (avg) if avg >= 0 else ' : invalid')
    complete_score = (single_scores['APE'] + single_scores['ARE'] + single_scores['RPE'] * 0.5 + single_scores['RRE'] * 0.5 +
                      single_scores['Badness'] + single_scores['InitQuality'] + single_scores['Robustness'] + single_scores['Relocalization Time']) / 6
    print('Final Score : %.4f' % complete_score)
