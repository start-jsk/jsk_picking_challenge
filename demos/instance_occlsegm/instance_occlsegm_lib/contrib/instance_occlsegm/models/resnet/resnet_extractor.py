import collections

import os.path as osp

import chainer
import chainer.functions as F
from chainer.links.model.vision.resnet import ResNet101Layers
from chainer.links.model.vision.resnet import ResNet50Layers

from chainer_mask_rcnn.models.resnet_extractor import _convert_bn_to_affine

import fcn


class ResNetExtractorBase(object):

    target_layer = 'res4'
    freeze_at = 'res2'

    def _init_layers(self, pretrained_model='auto', remove_layers=None):
        if pretrained_model == 'auto':
            # The pretrained weights are trained to accept BGR images.
            # Convert weights so that they accept RGB images.
            self.conv1.W.data[:] = self.conv1.W.data[:, ::-1]
        if remove_layers:
            # Remove no need layers to save memory
            for remove_layer in remove_layers:
                delattr(self, remove_layer)
                setattr(self, remove_layer, None)  # for the functions property
        _convert_bn_to_affine(self)

    @property
    def functions(self):
        return collections.OrderedDict([
            ('conv1', [self.conv1, self.bn1, F.relu]),
            ('pool1', [lambda x: F.max_pooling_2d(x, 3, stride=2)]),
            ('res2', [self.res2]),
            ('res3', [self.res3]),
            ('res4', [self.res4]),
            ('res5', [self.res5]),
        ])

    def __call__(self, x):
        if hasattr(self, 'mode'):
            raise RuntimeError(
                'mode attribute is deprecated, so please use freeze_at.')

        assert self.freeze_at is None or self.freeze_at in self.functions
        h = x
        for key, funcs in self.functions.items():
            for func in funcs:
                h = func(h)
            if key == self.freeze_at:
                h.unchain_backward()
            if key == self.target_layer:
                break
        return h


class ResNet50Extractor(ResNetExtractorBase, ResNet50Layers):

    def __init__(self, pretrained_model='auto', remove_layers=None):
        root = chainer.dataset.get_dataset_directory('pfnet/chainer/models')
        self.model_path = osp.join(root, 'ResNet-50-model.npz')
        if not osp.exists(self.model_path):
            self.download()

        super(ResNet50Extractor, self).__init__(pretrained_model)
        self._init_layers(pretrained_model, remove_layers)

    def download(self):
        url = 'https://drive.google.com/uc?id=1hSGnWZX_kjEWlfvi0fCHc8sczHio0i-t'  # NOQA
        md5 = '841b996a74049800cf0749ac97ab7eba'
        fcn.data.cached_download(url, self.model_path, md5)


class ResNet101Extractor(ResNetExtractorBase, ResNet101Layers):

    def __init__(self, pretrained_model='auto', remove_layers=None):
        root = chainer.dataset.get_dataset_directory('pfnet/chainer/models')
        self.model_path = osp.join(root, 'ResNet-101-model.npz')
        if not osp.exists(self.model_path):
            self.download()

        super(ResNet101Extractor, self).__init__(pretrained_model)
        self._init_layers(pretrained_model, remove_layers)

    def download(self):
        url = 'https://drive.google.com/uc?id=1c-wtuSDWmBCUTfNKLrQAIjrBMNMW4b7q'  # NOQA
        md5 = '2220786332e361fd7f956d9bf2f9d328'
        fcn.data.cached_download(url, self.model_path, md5)
