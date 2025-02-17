import copy

import chainer
from chainer import reporter
from chainercv.evaluations import eval_semantic_segmentation
from chainercv.utils import apply_to_iterator
import pandas
import six
import tqdm

from instance_occlsegm_lib.contrib.synthetic2d.evaluations import \
    eval_instseg_voc

from ..evaluations import eval_occlusion_segmentation


class PanopticSegmentationVOCEvaluator(chainer.training.extensions.Evaluator):

    name = 'validation'

    def __init__(self, iterator, target, device=None,
                 use_07_metric=False, label_names=None, show_progress=False):
        super(PanopticSegmentationVOCEvaluator, self).__init__(
            iterator=iterator, target=target, device=device)
        self.use_07_metric = use_07_metric
        self.label_names = label_names
        self._show_progress = show_progress

    def evaluate(self):
        target = self._targets['main']

        iterators = six.itervalues(self._iterators)
        total = len(self._iterators)
        if self._show_progress:
            iterators = tqdm.tqdm(iterators, total=total, leave=False)

        reports = []
        for iterator in iterators:
            report = self._evaluate_one(target, iterator)
            reports.append(report)
        report = pandas.DataFrame(reports).mean(skipna=True).to_dict()

        observation = dict()
        with reporter.report_scope(observation):
            reporter.report(report, target)
        return observation

    def _evaluate_one(self, target, iterator):
        if hasattr(iterator, 'reset'):
            iterator.reset()
            it = iterator
        else:
            it = copy.copy(iterator)

        if self._show_progress:
            it = tqdm.tqdm(it, total=len(it.dataset), leave=False)

        in_values, out_values, rest_values = apply_to_iterator(
            target.predict, it)

        imgs, = in_values
        pred_bboxes, pred_masks, pred_labels, pred_scores, \
            pred_lbls_vis, pred_lbls_occ = out_values

        gt_bboxes, gt_labels, gt_masks, gt_lbls_vis, gt_lbls_occ = rest_values

        # evaluate
        result = eval_instseg_voc(
            pred_masks, pred_labels, pred_scores,
            gt_masks, gt_labels, None,
            use_07_metric=self.use_07_metric,
        )

        result_lbl_vis = eval_semantic_segmentation(pred_lbls_vis, gt_lbls_vis)
        result['miou/vis'] = result_lbl_vis['miou']

        result_lbl_occ = eval_occlusion_segmentation(
            pred_lbls_occ, gt_lbls_occ
        )
        result['miou/occ'] = result_lbl_occ['miou']

        result['miou'] = (result['miou/vis'] + result['miou/occ']) / 2.

        return result
